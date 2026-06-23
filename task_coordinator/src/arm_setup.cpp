#include "task_coordinator/arm_setup.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <fstream>
#include <cmath>
#include <sstream>
#include <vector>

namespace
{
std::string readFile(const std::string & path)
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

// Translates MoveIt's error code into a short, specific reason - lets
// callers distinguish e.g. "no IK solution for this pose" from "found a
// plan but the goal is in collision" from "execution failed", instead of
// just a flat "failed".
std::string describeMoveItError(const moveit::core::MoveItErrorCode & code)
{
  using moveit::core::MoveItErrorCode;
  switch (code.val) {
    case MoveItErrorCode::PLANNING_FAILED:
      return "no valid path found";
    case MoveItErrorCode::NO_IK_SOLUTION:
      return "no IK solution for target pose";
    case MoveItErrorCode::INVALID_MOTION_PLAN:
      return "invalid motion plan";
    case MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "plan invalidated by environment change";
    case MoveItErrorCode::CONTROL_FAILED:
      return "controller failed during execution";
    case MoveItErrorCode::TIMED_OUT:
      return "timed out";
    case MoveItErrorCode::PREEMPTED:
      return "preempted by another command";
    case MoveItErrorCode::START_STATE_IN_COLLISION:
      return "start state in collision";
    case MoveItErrorCode::GOAL_IN_COLLISION:
      return "target pose is in collision";
    case MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
      return "goal violates path constraints";
    case MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
      return "goal constraints violated";
    case MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
      return "invalid goal constraints (target pose likely unreachable)";
    case MoveItErrorCode::FAILURE:
      // MoveIt's generic catch-all - typically means the OMPL planner
      // couldn't find any valid path/IK solution for the target pose
      // within its allotted attempts, without a more specific reason.
      return "no valid path/IK solution found - target pose likely unreachable";
    default:
      return "MoveIt error code " + std::to_string(code.val);
  }
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
  const std::string & label,
  std::string * error_out)
{
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto plan_result = group.plan(plan);
  if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
    std::string reason = describeMoveItError(plan_result);
    RCLCPP_ERROR(logger, "%s: planning failed - %s", label.c_str(), reason.c_str());
    if (error_out) {
      *error_out = "planning failed: " + reason;
    }
    return false;
  }
  auto exec_result = group.execute(plan);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    std::string reason = describeMoveItError(exec_result);
    RCLCPP_ERROR(logger, "%s: execution failed - %s", label.c_str(), reason.c_str());
    if (error_out) {
      *error_out = "execution failed: " + reason;
    }
    return false;
  }
  RCLCPP_INFO(logger, "%s: done", label.c_str());
  return true;
}

namespace
{
void rotateVectorByQuaternion(
  double vx,
  double vy,
  double vz,
  const geometry_msgs::msg::Quaternion & q_msg,
  double & ox,
  double & oy,
  double & oz)
{
  double x = q_msg.x;
  double y = q_msg.y;
  double z = q_msg.z;
  double w = q_msg.w;

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm > 1e-12) {
    x /= norm;
    y /= norm;
    z /= norm;
    w /= norm;
  } else {
    ox = vx;
    oy = vy;
    oz = vz;
    return;
  }

  // Rotation matrix for q * v * q^-1.
  ox = (1.0 - 2.0 * (y * y + z * z)) * vx +
       (2.0 * (x * y - z * w)) * vy +
       (2.0 * (x * z + y * w)) * vz;
  oy = (2.0 * (x * y + z * w)) * vx +
       (1.0 - 2.0 * (x * x + z * z)) * vy +
       (2.0 * (y * z - x * w)) * vz;
  oz = (2.0 * (x * z - y * w)) * vx +
       (2.0 * (y * z + x * w)) * vy +
       (1.0 - 2.0 * (x * x + y * y)) * vz;
}
}  // namespace

bool jogCartesian(
  moveit::planning_interface::MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label,
  double dx,
  double dy,
  double dz,
  bool in_tool_frame,
  std::string * error_out)
{
  geometry_msgs::msg::Pose target = group.getCurrentPose().pose;

  double wx = dx;
  double wy = dy;
  double wz = dz;
  if (in_tool_frame) {
    rotateVectorByQuaternion(dx, dy, dz, target.orientation, wx, wy, wz);
  }

  target.position.x += wx;
  target.position.y += wy;
  target.position.z += wz;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target);

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes cart_error;
  const double kJogEefStep = 0.001;  // 1 mm IK/collision-check resolution.

  const double fraction = group.computeCartesianPath(
    waypoints, kJogEefStep, trajectory, true, &cart_error);

  if (fraction < 0.999) {
    const std::string reason = (fraction < 0.0)
      ? describeMoveItError(cart_error)
      : "only " + std::to_string(static_cast<int>(fraction * 100.0)) +
        "% of the jog step was reachable";
    RCLCPP_ERROR(logger, "%s: Cartesian path failed - %s", label.c_str(), reason.c_str());
    if (error_out) {
      *error_out = "jog failed: " + reason;
    }
    return false;
  }

  auto exec_result = group.execute(trajectory);
  if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
    std::string reason = describeMoveItError(exec_result);
    RCLCPP_ERROR(logger, "%s: execution failed - %s", label.c_str(), reason.c_str());
    if (error_out) {
      *error_out = "execution failed: " + reason;
    }
    return false;
  }

  RCLCPP_INFO(
    logger, "%s: done (%s frame)", label.c_str(), in_tool_frame ? "tool" : "world");
  return true;
}


}  // namespace task_coordinator
