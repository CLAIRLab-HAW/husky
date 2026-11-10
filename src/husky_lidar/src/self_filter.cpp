// self_filter: the lidar's points without the robot's own body.
//
// The RS16 on the a200's sensor arch sees the arch beside it and an arm reaching out in front.  The arch stands inside
// the footprint, which the costmaps clear; the arm does not, and in the costmaps it stood as an obstacle right ahead
// of the robot -- "RegulatedPurePursuitController detected collision ahead!" on every drive with the arm out, and a
// trail of marks along its way (2026-09-27, ManiSkill stack).  This node drops every point inside one of the robot's
// collision bodies (robot_body.hpp: the live description, the links where TF has them at the cloud's stamp) and
// publishes the rest on ``points_filtered`` -- what the costmaps and pointcloud_to_laserscan read.  Until the
// description has come, it passes every point through: an unfiltered scan beats none.
//
// Why C++: the Python node cost 12.5 % of a core in the ManiSkill stack (2026-09-27), and the robot's lidar runs it
// on every sweep.
//
// Its names are relative (``points``, ``points_filtered``, ``robot_description``); launch/self_filter.launch.py maps
// them onto husky_lidar.wiring's::
//
//     ros2 launch husky_lidar self_filter.launch.py use_sim_time:=true

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

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
        [this](const std_msgs::msg::String& msg) { onDescription(msg); });
    points_ = create_subscription<PointCloud2>("points", rclcpp::SensorDataQoS(),
                                               [this](const PointCloud2& msg) { onPoints(msg); });
    RCLCPP_INFO(get_logger(), "%s without the robot's body -> %s", points_->get_topic_name(),
                publisher_->get_topic_name());
  }

private:
  void onDescription(const std_msgs::msg::String& msg)
  {
    if (msg.data.empty() || msg.data == urdf_)
      return;
    try
    {
      body_ = std::make_unique<RobotBody>(msg.data, kPaddingM);
    }
    catch (const std::runtime_error& e)
    {
      RCLCPP_ERROR(get_logger(), "%s -- the body stays as it was", e.what());
      return;
    }
    urdf_ = msg.data;
    RCLCPP_INFO(get_logger(), "the robot's body: %zu bodies on %zu links, dropped from the lidar", body_->size(),
                body_->links().size());
  }

  Poses poses(const std::string& frame, const builtin_interfaces::msg::Time& stamp) const
  {
    Poses found;
    for (const auto& link : body_->links())
    {
      geometry_msgs::msg::TransformStamped t;
      try
      {
        t = tf_buffer_->lookupTransform(frame, link, rclcpp::Time(stamp));
      }
      catch (const tf2::TransformException&)
      {
        try  // the cloud's stamp not in the buffer yet: the latest the arm stood at
        {
          t = tf_buffer_->lookupTransform(frame, link, tf2::TimePointZero);
        }
        catch (const tf2::TransformException&)
        {
          continue;
        }
      }
      found.emplace(link, tf2::transformToEigen(t));
    }
    return found;
  }

  void onPoints(const PointCloud2& msg)
  {
    if (!body_)
    {
      publisher_->publish(msg);
      return;
    }
    const std::size_t n = std::size_t(msg.width) * msg.height;
    std::vector<Eigen::Vector3d> xyz;
    xyz.reserve(n);
    sensor_msgs::PointCloud2ConstIterator<float> x(msg, "x"), y(msg, "y"), z(msg, "z");
    for (std::size_t i = 0; i < n; ++i, ++x, ++y, ++z)
      xyz.emplace_back(*x, *y, *z);
    const auto drop = body_->robotPoints(xyz, poses(msg.header.frame_id, msg.header.stamp));

    PointCloud2 out;
    out.header = msg.header;
    out.fields = msg.fields;
    out.is_bigendian = msg.is_bigendian;
    out.point_step = msg.point_step;
    out.is_dense = msg.is_dense;
    out.height = 1;
    out.data.reserve(msg.data.size());
    for (std::size_t i = 0; i < n; ++i)
      if (!drop[i])
      {
        const auto* point = msg.data.data() + i * msg.point_step;
        out.data.insert(out.data.end(), point, point + msg.point_step);
      }
    out.width = out.data.size() / msg.point_step;
    out.row_step = out.data.size();
    publisher_->publish(out);

    dropped_ += n - out.width;
    seen_ += n;
    if (std::chrono::steady_clock::now() - said_ > std::chrono::minutes(1))
    {
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<husky_lidar::SelfFilter>());
  rclcpp::shutdown();
  return 0;
}
