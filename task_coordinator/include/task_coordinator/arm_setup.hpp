#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <memory>
#include <string>

namespace task_coordinator
{

// Node + params (BEFORE the executor spins).
// The robot_description parameter name is declared as
// "<robot_ns>_robot_description" (not "robot_description") - RobotModelLoader
// caches robot models process-wide keyed by this string; with two robots
// using the same name, the second would get back the first one's model.
rclcpp::Node::SharedPtr makeArmNode(
  const std::string & node_name,
  const std::string & robot_ns,
  const std::string & description_pkg,
  const std::string & urdf_rel_path,
  const std::string & moveit_pkg,
  const std::string & srdf_rel_path,
  const std::string & group_name = "robot");

// MoveGroupInterface (AFTER the node is spinning).
std::unique_ptr<moveit::planning_interface::MoveGroupInterface> makeMoveGroup(
  const rclcpp::Node::SharedPtr & node,
  const std::string & robot_ns,
  const std::string & group_name = "robot",
  const std::string & pipeline   = "ompl");

// Plans and executes whatever target was already set on group via
// setNamedTarget/setJointValueTarget/setPoseTarget etc. The caller only sets
// the target - this part (plan -> execute -> logging) is the same for any
// kind of target.
bool planAndExecute(
  moveit::planning_interface::MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label);

}  // namespace task_coordinator
