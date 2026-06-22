#include "task_coordinator/arm_setup.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <sstream>

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

namespace task_coordinator
{

rclcpp::Node::SharedPtr makeArmNode(
  const std::string & node_name,
  const std::string & robot_ns,
  const std::string & description_pkg,
  const std::string & urdf_rel_path,
  const std::string & moveit_pkg,
  const std::string & srdf_rel_path,
  const std::string & group_name)
{
  // use_sim_time has to be set via NodeOptions at construction time, not
  // declared after the fact - rclcpp::Node's constructor checks it right
  // away to decide whether to wire up an internal /clock subscription.
  // Without it, this node's own clock stays on wall time even though the
  // rest of the system (move_group, joint_states, ...) runs on sim time,
  // which silently breaks any client-side "current state" fetch (e.g.
  // MoveGroupInterface::getCurrentPose()) - it compares a wall-time "now"
  // against sim-timestamped messages and always finds them "too old".
  // plan()/execute() are unaffected since those are forwarded as RPCs to
  // the move_group action server, which already has correct sim time.
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = rclcpp::Node::make_shared(node_name, robot_ns, options);

  const std::string description_base = robot_ns + "_robot_description";

  // Declared directly (not left to RDFLoader's topic fallback): the
  // fallback only listens on a topic matching the literal param name, and
  // that name has to be distinct per robot to avoid colliding in
  // RobotModelLoader's process-wide model cache - so it can't be the
  // "robot_description" name robot_state_publisher actually publishes on.
  node->declare_parameter(
    description_base,
    readFile(
      ament_index_cpp::get_package_share_directory(description_pkg) + "/" + urdf_rel_path));

  node->declare_parameter(
    description_base + "_semantic",
    readFile(
      ament_index_cpp::get_package_share_directory(moveit_pkg) + "/" + srdf_rel_path));

  node->declare_parameter(
    description_base + "_kinematics." + group_name + ".kinematics_solver",
    std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
  node->declare_parameter(
    description_base + "_kinematics." + group_name + ".kinematics_solver_search_resolution", 0.005);
  node->declare_parameter(
    description_base + "_kinematics." + group_name + ".kinematics_solver_timeout", 0.005);

  return node;
}

std::unique_ptr<moveit::planning_interface::MoveGroupInterface> makeMoveGroup(
  const rclcpp::Node::SharedPtr & node,
  const std::string & robot_ns,
  const std::string & group_name,
  const std::string & pipeline)
{
  using moveit::planning_interface::MoveGroupInterface;

  MoveGroupInterface::Options options(group_name, robot_ns + "_robot_description", "/" + robot_ns);
  auto move_group = std::make_unique<MoveGroupInterface>(node, options);
  move_group->setPlanningPipelineId(pipeline);
  return move_group;
}

bool planAndExecute(
  moveit::planning_interface::MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: planning failed", label.c_str());
    return false;
  }
  if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "%s: execution failed", label.c_str());
    return false;
  }
  RCLCPP_INFO(logger, "%s: done", label.c_str());
  return true;
}

}  // namespace task_coordinator
