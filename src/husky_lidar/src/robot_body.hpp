/**
 * The robot's body as geometric_shapes bodies: which points of a sensor are the robot itself.
 *
 * Each <collision> of the robot description becomes a body in its link's frame, grown by a padding: a primitive as it
 * is, a mesh as its convex hull (bodies::ConvexMesh).  Where the links stand comes from the caller (TF in the self
 * filter).  A convex hull is looser than the triangles; the chassis' hull fills the space above its deck, which is the
 * robot's own anyway.
 *
 * robot_body_filter would be the package to take, but has no ROS 2 port: its `ros2` branch is the catkin package with
 * one CI commit on top, unreleased for Jazzy (checked 2026-09-27).  geometric_shapes is what it builds on.
 */

#ifndef ROBOT_BODY_HPP_
#define ROBOT_BODY_HPP_

#include <algorithm>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Geometry"
#include "geometric_shapes/bodies.h"
#include "geometric_shapes/body_operations.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shapes.h"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace husky_lidar
{
/**
 * How far from the sensor a point can still be the robot's [m]: the RS16 sits on the rear of the arch, and the arm
 * reaches some 1.6 m ahead of it.  Only the points nearer than this go through the bodies.
 */
constexpr double kNearM = 2.2;
/// How far around a link a point still counts as the robot [m]: the RS16's centimeter noise and a link a TF tick off.
constexpr double kPaddingM = 0.05;
/**
 * The arm's links -- the UR5, the gripper, the wrist camera -- get this padding instead [m].  Its cable lies off the
 * model, and not where the description would put it: rg6_cable has no collision body, and the RS16 kept 12 points a
 * sweep near the arm in `forward`; in the look poses they fell into the 2D scan and SLAM mapped them around every place
 * the robot looked from (a200-0553, 2026-11-23).  0.12 m around the forearm, wrist and gripper left 1; the whole arm
 * gets 0.15 m, as the owner asked for a margin.  What stands within it of the arm is dropped with it -- harmless for
 * driving, where the arm stands in the footprint, and MoveIt plans from the D435's octomap.
 */
constexpr double kArmPaddingM = 0.15;
/// The arm's links, by name prefix.
inline const std::vector<std::string> kArmLinks = {"arm_0_", "rg6_", "camera_0"};

/**
 * A box around the sensor arch and what hangs off it, fixed to the lidar's link -- dropped like a robot body.  On the
 * a200-0553 ~2 points a sweep stayed at x -0.28, y -0.38..-0.57, z 0.65..0.70 m in base_link, on the arch's right and
 * past the base's half width, where no link of the description stands (2026-11-23): something on the arch the model
 * does not carry, mapped beside the robot wherever it stood.  The box reaches 0.65 m to either side and from 0.43 to
 * 0.98 m up; what stands there at the robot's rear is dropped with it. Its center in the lidar link's frame [m] (the
 * lidar sits at x -0.28, z 0.75 in base_link) and its size [m].
 */
inline const Eigen::Vector3d kArchBoxCenterM{0.0, -0.01, -0.05};
inline const Eigen::Vector3d kArchBoxSizeM{0.35, 1.30, 0.55};
/// The link the box is fixed to.
inline const std::string kArchBoxLink = "lidar3d_0_link";

inline double paddingFor(const std::string & link, double padding_m)
{
  const bool arm = std::ranges::any_of(kArmLinks, [&](const std::string & prefix) {return link.starts_with(prefix);});
  return arm ? std::max(padding_m, kArmPaddingM) : padding_m;
}

/// Each link's pose in the points' frame.
using Poses = std::map<std::string, Eigen::Isometry3d>;

class RobotBody
{
public:
  /// Throws std::runtime_error on a description urdf cannot parse.
  RobotBody(const std::string & urdf, double padding_m)
  {
    urdf::Model model;
    if (!model.initString(urdf)) {
      throw std::runtime_error("the robot description does not parse");
    }
    for (const auto & [name, link] : model.links_) {
      for (const auto & collision : link->collision_array) {
        const auto shape = toShape(*collision->geometry);
        if (!shape) {
          continue;
        }
        // shortcut: a flat mesh (a plate) has no hull and stays in the scan; the Python filter took its bounding
        // sphere.  Upgrade if a plate of the robot shows up in points_filtered.
        std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(shape.get()));
        body->setPadding(paddingFor(name, padding_m));
        parts_.push_back({name, toIsometry(collision->origin), std::move(body)});
        links_.insert(name);
      }
    }
    if (model.links_.contains(kArchBoxLink)) {
      const shapes::Box box(kArchBoxSizeM.x(), kArchBoxSizeM.y(), kArchBoxSizeM.z());
      std::unique_ptr<bodies::Body> body(bodies::createBodyFromShape(&box));
      body->setPadding(padding_m);
      parts_.push_back({kArchBoxLink, Eigen::Isometry3d(Eigen::Translation3d(kArchBoxCenterM)), std::move(body)});
      links_.insert(kArchBoxLink);
    }
  }

  std::size_t size() const {return parts_.size();}
  const std::set<std::string> & links() const {return links_;}

  /**
   * Which of the points `xyz` (in the sensor's frame) are the robot's: nearer than `near_m` and inside a body whose
   * link has a pose in `poses`; a link without one is left out.
   */
  std::vector<bool> robotPoints(const std::vector<Eigen::Vector3d> & xyz, const Poses & poses, double near_m = kNearM)
  {
    std::vector<const bodies::Body *> posed;
    for (auto & part : parts_) {
      if (auto pose = poses.find(part.link); pose != poses.end()) {
        part.body->setPose(pose->second * part.origin);
        posed.push_back(part.body.get());
      }
    }
    std::vector<bool> drop;
    drop.reserve(xyz.size());
    for (const auto & point : xyz) {
      drop.push_back(
        point.squaredNorm() < near_m * near_m &&
        std::ranges::any_of(posed, [&](const bodies::Body * body) {return body->containsPoint(point);}));
    }
    return drop;
  }

private:
  struct Part
  {
    std::string link;
    Eigen::Isometry3d origin;  // the collision in its link's frame
    std::unique_ptr<bodies::Body> body;
  };

  static std::unique_ptr<shapes::Shape> toShape(const urdf::Geometry & geometry)
  {
    switch (geometry.type) {
      case urdf::Geometry::BOX: {
          const auto & box = static_cast<const urdf::Box &>(geometry);
          return std::make_unique<shapes::Box>(box.dim.x, box.dim.y, box.dim.z);
        }
      case urdf::Geometry::CYLINDER: {
          const auto & cylinder = static_cast<const urdf::Cylinder &>(geometry);
          return std::make_unique<shapes::Cylinder>(cylinder.radius, cylinder.length);
        }
      case urdf::Geometry::SPHERE:
        return std::make_unique<shapes::Sphere>(static_cast<const urdf::Sphere &>(geometry).radius);
      case urdf::Geometry::MESH: {
          const auto & mesh = static_cast<const urdf::Mesh &>(geometry);
          // package:// and file:// through resource_retriever, the ament index behind it.
          std::unique_ptr<shapes::Shape> shape(shapes::createMeshFromResource(
              mesh.filename, Eigen::Vector3d(mesh.scale.x, mesh.scale.y, mesh.scale.z)));
          if (!shape) {
            RCLCPP_WARN(rclcpp::get_logger("self_filter"), "no mesh for %s -- that part of the robot stays in the scan",
                      mesh.filename.c_str());
          }
          return shape;
        }
    }
    return nullptr;
  }

  static Eigen::Isometry3d toIsometry(const urdf::Pose & pose)
  {
    const auto & p = pose.position;
    const auto & q = pose.rotation;
    return Eigen::Translation3d(p.x, p.y, p.z) * Eigen::Quaterniond(q.w, q.x, q.y, q.z);
  }

  std::vector<Part> parts_;
  std::set<std::string> links_;
};
}  // namespace husky_lidar

#endif  // ROBOT_BODY_HPP_
