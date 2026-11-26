/**
 * The self filter drops the lidar's points of the robot itself, and only near the sensor.
 */

#include <gtest/gtest.h>

#include "robot_body.hpp"

namespace
{
constexpr char kArm[] =
  R"(<robot name="r"><link name="arm"><collision><origin xyz="0.5 0 0" rpy="0 0 0"/>
  <geometry><box size="1.0 0.2 0.2"/></geometry></collision></link></robot>)";

const husky_lidar::Poses kAtSensor{{"arm", Eigen::Isometry3d::Identity()}};
}  // namespace

TEST(RobotBody, TakesThePointsInsideAndThePaddingAroundIt)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  const auto drop = body.robotPoints({{0.8, 0.0, 0.0}, {0.8, 0.14, 0.0}, {0.8, 0.5, 0.0}}, kAtSensor);
  EXPECT_EQ(drop, std::vector<bool>({true, true, false})) << "inside, padding, beside";
}

TEST(RobotBody, FarPointsAreNeverTheRobot)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  const auto drop = body.robotPoints({{0.8, 0.0, 0.0}}, kAtSensor, 0.5);
  EXPECT_EQ(drop, std::vector<bool>({false})) << "a point beyond near_m costs no body test and stays";
}

TEST(RobotBody, ALinkTfDoesNotReachFiltersNothing)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  EXPECT_EQ(body.robotPoints({{0.8, 0.0, 0.0}}, {}), std::vector<bool>({false}));
}

TEST(RobotBody, MovesWithItsLink)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  husky_lidar::Poses turned{{"arm", Eigen::Isometry3d(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()))}};
  EXPECT_EQ(body.robotPoints({{0.0, 0.8, 0.0}, {0.8, 0.0, 0.0}}, turned), std::vector<bool>({true, false}));
}

TEST(RobotBody, RefusesADescriptionThatDoesNotParse)
{
  EXPECT_THROW(husky_lidar::RobotBody("<robot", 0.05), std::runtime_error);
}

TEST(RobotBody, ArmGripperAndCameraPrefixesGetTheCableMarginWithoutGrowingTheChassis)
{
  for (const auto & link : {"arm_0_forearm_link", "rg6_body", "camera_0_link", "chassis"}) {
    SCOPED_TRACE(link);
    const std::string description = std::string("<robot name=\"r\"><link name=\"") + link +
      "\"><collision><origin xyz=\"0.5 0 0\"/><geometry><box size=\"1.0 0.2 0.2\"/>"
      "</geometry></collision></link></robot>";
    const husky_lidar::Poses poses{{link, Eigen::Isometry3d::Identity()}};
    husky_lidar::RobotBody body(description, 0.05);
    const bool arm_margin = std::string(link) != "chassis";
    EXPECT_EQ(body.robotPoints({{0.8, 0.22, 0.0}, {0.8, 0.30, 0.0}}, poses),
              std::vector<bool>({arm_margin, false}));
    husky_lidar::RobotBody wider(description, 0.30);
    EXPECT_EQ(wider.robotPoints({{0.8, 0.35, 0.0}}, poses), std::vector<bool>({true}))
      << "the requested padding must not be narrowed to the arm's minimum";
  }
}

TEST(RobotBody, TheSyntheticArchBodyNeedsItsLidarLinkAndFollowsItsPose)
{
  husky_lidar::RobotBody body("<robot name=\"r\"><link name=\"lidar3d_0_link\"/></robot>", 0.05);
  EXPECT_EQ(body.size(), 1U) << "the arch has a body even without an authored collision";
  const std::vector<Eigen::Vector3d> points{{0.0, 0.60, 0.0}, {0.0, 0.0, 0.4}};
  EXPECT_EQ(body.robotPoints(points, {}), std::vector<bool>({false, false}));
  const husky_lidar::Poses at_sensor{{"lidar3d_0_link", Eigen::Isometry3d::Identity()}};
  EXPECT_EQ(body.robotPoints(points, at_sensor), std::vector<bool>({true, false}));
  const husky_lidar::Poses moved{
    {"lidar3d_0_link", Eigen::Isometry3d(Eigen::Translation3d(1.0, 0.0, 0.0))}
  };
  EXPECT_EQ(body.robotPoints({points[0], {1.0, 0.60, 0.0}}, moved), std::vector<bool>({false, true}));
}
