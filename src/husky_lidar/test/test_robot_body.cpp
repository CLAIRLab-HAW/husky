// The self filter drops the lidar's points of the robot itself, and only near the sensor.

#include <gtest/gtest.h>

#include "robot_body.hpp"

namespace
{
constexpr char kArm[] = R"(<robot name="r"><link name="arm"><collision><origin xyz="0.5 0 0" rpy="0 0 0"/>
  <geometry><box size="1.0 0.2 0.2"/></geometry></collision></link></robot>)";

const husky_lidar::Poses kAtSensor{ { "arm", Eigen::Isometry3d::Identity() } };
}  // namespace

TEST(RobotBody, TakesThePointsInsideAndThePaddingAroundIt)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  const auto drop = body.robotPoints({ { 0.8, 0.0, 0.0 }, { 0.8, 0.14, 0.0 }, { 0.8, 0.5, 0.0 } }, kAtSensor);
  EXPECT_EQ(drop, std::vector<bool>({ true, true, false })) << "inside, padding, beside";
}

TEST(RobotBody, FarPointsAreNeverTheRobot)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  const auto drop = body.robotPoints({ { 0.8, 0.0, 0.0 } }, kAtSensor, 0.5);
  EXPECT_EQ(drop, std::vector<bool>({ false })) << "a point beyond near_m costs no body test and stays";
}

TEST(RobotBody, ALinkTfDoesNotReachFiltersNothing)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  EXPECT_EQ(body.robotPoints({ { 0.8, 0.0, 0.0 } }, {}), std::vector<bool>({ false }));
}

TEST(RobotBody, MovesWithItsLink)
{
  husky_lidar::RobotBody body(kArm, 0.05);
  husky_lidar::Poses turned{ { "arm", Eigen::Isometry3d(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())) } };
  EXPECT_EQ(body.robotPoints({ { 0.0, 0.8, 0.0 }, { 0.8, 0.0, 0.0 } }, turned), std::vector<bool>({ true, false }));
}

TEST(RobotBody, RefusesADescriptionThatDoesNotParse)
{
  EXPECT_THROW(husky_lidar::RobotBody("<robot", 0.05), std::runtime_error);
}
