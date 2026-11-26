/**
 * self_filter: the lidar's points without the robot's own body.
 *
 * The RS16 on the a200's sensor arch sees the arch beside it and an arm reaching out in front.  The arch stands inside
 * the footprint, which the costmaps clear; the arm does not, and in the costmaps it stood as an obstacle right ahead of
 * the robot -- "RegulatedPurePursuitController detected collision ahead!" on every drive with the arm out, and a trail
 * of marks along its way (2026-09-27, ManiSkill stack).  This node drops every point inside one of the robot's
 * collision bodies (robot_body.hpp: the live description, the links where TF has them at the cloud's stamp) and
 * publishes the rest on `points_filtered` -- what the costmaps and pointcloud_to_laserscan read.  Until the description
 * has come, it passes every point through: an unfiltered scan beats none.
 *
 * Why C++: the Python node cost 12.5 % of a core in the ManiSkill stack (2026-09-27), and the robot's lidar runs it on
 * every sweep.
 *
 * Its names are relative (`points`, `points_filtered`, `robot_description`); launch/self_filter.launch.py maps them
 * onto husky_lidar.wiring's:
 *
 *     ros2 launch husky_lidar self_filter.launch.py use_sim_time:=true
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <memory>
#include <ranges>  // NOLINT(build/include_order): Jazzy's cpplint predates the C++20 headers
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "robot_body.hpp"

namespace husky_lidar
{
using sensor_msgs::msg::PointCloud2;

class SelfFilter : public rclcpp::Node
{
public:
  SelfFilter()
  : Node("self_filter")
    , tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock()))
    , tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
  {
    publisher_ = create_publisher<PointCloud2>("points_filtered", 1);
    description_ = create_subscription<std_msgs::msg::String>(
        "robot_description", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String & msg) {onDescription(msg);});
    points_ = create_subscription<PointCloud2>("points", rclcpp::SensorDataQoS(),
        [this](const PointCloud2 & msg) {onPoints(msg);});
    RCLCPP_INFO(get_logger(), "%s without the robot's body -> %s", points_->get_topic_name(),
                publisher_->get_topic_name());
  }

private:
  void onDescription(const std_msgs::msg::String & msg)
  {
    if (msg.data.empty() || msg.data == urdf_) {
      return;
    }
    try {
      body_ = std::make_unique<RobotBody>(msg.data, kPaddingM);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(get_logger(), "%s -- the body stays as it was", e.what());
      return;
    }
    urdf_ = msg.data;
    RCLCPP_INFO(get_logger(), "the robot's body: %zu bodies on %zu links, dropped from the lidar", body_->size(),
                body_->links().size());
  }

  Poses poses(const std::string & frame, const builtin_interfaces::msg::Time & stamp) const
  {
    Poses found;
    const tf2::TimePoint stamped = tf2_ros::fromMsg(stamp);
    for (const auto & link : body_->links()) {
      // The cloud's stamp not in the buffer yet: the latest the link stood at.  Asked rather than caught: at 10 Hz
      // and some 30 links, an exception per link and cloud is the expensive way to learn the same thing.
      const tf2::TimePoint when = tf_buffer_->canTransform(frame, link, stamped) ? stamped : tf2::TimePointZero;
      try {
        found.emplace(link, tf2::transformToEigen(tf_buffer_->lookupTransform(frame, link, when)));
      } catch (const tf2::TransformException &) {
        continue;  // no transform at all yet, or the buffer moved on between the two calls
      }
    }
    return found;
  }

  /**
   * Why `msg` cannot be filtered, or nullptr when it can: x, y and z as float32 inside each point, and rows without
   * padding, which is what the copy below steps through.  The iterator throws on a missing field, and an exception out
   * of this callback ends the node.
   */
  static const char * unfilterable(const PointCloud2 & msg)
  {
    for (const char * axis : {"x", "y", "z"}) {
      const auto field = std::ranges::find(msg.fields, axis, &sensor_msgs::msg::PointField::name);
      if (field == msg.fields.end()) {
        return "it has no x, y and z";
      }
      if (field->datatype != sensor_msgs::msg::PointField::FLOAT32 || field->offset + 4 > msg.point_step) {
        return "its x, y and z are not float32 inside a point";
      }
    }
    if (std::size_t(msg.width) * msg.point_step != msg.row_step ||
      msg.data.size() < std::size_t(msg.row_step) * msg.height)
    {
      return "its rows are padded or its data is short";
    }
    return nullptr;
  }

  void onPoints(const PointCloud2 & msg)
  {
    if (!body_) {
      publisher_->publish(msg);
      return;
    }
    // A cloud that cannot be filtered goes through as it came, as before the description: an unfiltered scan beats
    // none.
    if (const char * why = unfilterable(msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "a cloud on %s passes unfiltered: %s",
        points_->get_topic_name(), why);
      publisher_->publish(msg);
      return;
    }
    const auto xyz = pointsOf(msg);
    const auto found = poses(msg.header.frame_id, msg.header.stamp);
    if (found.size() < body_->links().size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
                           "%zu of %zu body links have no transform to %s -- their points stay in",
                           body_->links().size() - found.size(), body_->links().size(), msg.header.frame_id.c_str());
    }
    const auto out = kept(msg, body_->robotPoints(xyz, found));
    publisher_->publish(out);
    tally(xyz.size(), xyz.size() - out.width);
  }

  /// The cloud's points as vectors, in its own frame.
  static std::vector<Eigen::Vector3d> pointsOf(const PointCloud2 & msg)
  {
    const std::size_t n = std::size_t(msg.width) * msg.height;
    std::vector<Eigen::Vector3d> xyz;
    xyz.reserve(n);
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    for (std::size_t i = 0; i < n; ++i, ++x, ++y, ++z) {
      xyz.emplace_back(*x, *y, *z);
    }
    return xyz;
  }

  /// `msg` with the points `drop` marks left out, every field of the rest as it was, in one row.
  static PointCloud2 kept(const PointCloud2 & msg, const std::vector<bool> & drop)
  {
    PointCloud2 out;
    out.header = msg.header;
    out.fields = msg.fields;
    out.is_bigendian = msg.is_bigendian;
    out.point_step = msg.point_step;
    out.is_dense = msg.is_dense;
    out.height = 1;
    out.data.reserve(msg.data.size());
    for (const auto & [dropped, point] : std::views::zip(drop, msg.data | std::views::chunk(msg.point_step))) {
      if (!dropped) {
        out.data.insert(out.data.end(), point.begin(), point.end());
      }
    }
    out.width = out.data.size() / msg.point_step;
    out.row_step = out.data.size();
    return out;
  }

  /// Once a minute, how many of the points seen were the robot's.
  void tally(std::size_t seen, std::size_t dropped)
  {
    dropped_ += dropped;
    seen_ += seen;
    if (std::chrono::steady_clock::now() - said_ > std::chrono::minutes(1)) {
      RCLCPP_INFO(get_logger(), "%zu of %zu points were the robot's in the last minute", dropped_, seen_);
      dropped_ = seen_ = 0;
      said_ = std::chrono::steady_clock::now();
    }
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_;
  rclcpp::Subscription<PointCloud2>::SharedPtr points_;
  std::unique_ptr<RobotBody> body_;
  std::string urdf_;
  std::size_t dropped_ = 0;
  std::size_t seen_ = 0;
  std::chrono::steady_clock::time_point said_ = std::chrono::steady_clock::now();
};
}  // namespace husky_lidar

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<husky_lidar::SelfFilter>());
  rclcpp::shutdown();
  return 0;
}
