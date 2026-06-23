#include "task_coordinator/arm_setup.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <task_coordinator/srv/select_pose.hpp>
#include <task_coordinator/srv/move_to_pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;
using task_coordinator::makeArmNode;
using task_coordinator::makeMoveGroup;
using task_coordinator::planAndExecute;

namespace
{
bool goNamedHome(
  MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label,
  std::string & error_out)
{
  group.setNamedTarget("home_pose");
  return planAndExecute(group, logger, label, &error_out);
}

bool goJointTargets(
  MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label,
  const std::vector<double> & joint_targets,
  std::string & error_out)
{
  if (!group.setJointValueTarget(joint_targets)) {
    error_out = "joint targets out of bounds or invalid size";
    RCLCPP_ERROR(logger, "%s: %s", label.c_str(), error_out.c_str());
    return false;
  }
  return planAndExecute(group, logger, label, &error_out);
}

bool goPoseTarget(
  MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label,
  const geometry_msgs::msg::Pose & pose,
  std::string & error_out)
{
  group.setPoseTarget(pose);
  return planAndExecute(group, logger, label, &error_out);
}

// Named joint poses, carried over from sim_task.cpp's hardcoded joint
// targets (placeholders - known to be inside each robot's workspace).
// Selecting one moves there directly - no chaining between poses.
const std::map<std::string, std::vector<double>> kBeanPoses = {
  {"pose_1", {-0.174, 0.384, 1.309, -1.326, 1.501, -2.25}},
  {"pose_2", {-0.506, 0.436, -0.523, -0.733, -0.576, -1.826}},
};
const std::map<std::string, std::vector<double>> kNemoPoses = {
  {"pose_1", {-0.907, -0.192, -0.174, -2.077, 0.716, 0.628, -0.506}},
};

bool selectNamedPose(
  MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label,
  const std::map<std::string, std::vector<double>> & poses,
  const std::string & pose_name,
  std::string & error_out)
{
  auto it = poses.find(pose_name);
  if (it == poses.end()) {
    error_out = "Unknown pose: " + pose_name;
    return false;
  }
  return goJointTargets(group, logger, label, it->second, error_out);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto bean_node = makeArmNode(
    "hmi_motion_bean", "bean", "bean3_description", "urdf/bean2.urdf",
    "bean2_moveit", "config/bean2.srdf", "robot_bean");
  auto nemo_node = makeArmNode(
    "hmi_motion_nemo", "nemo", "nemo1_description", "urdf/nemo1.urdf",
    "nemo1_moveit", "config/nemo1.srdf", "robot_nemo");
  auto service_node = rclcpp::Node::make_shared("hmi_motion_server");

  auto logger = rclcpp::get_logger("hmi_motion_server");

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(bean_node);
  executor.add_node(nemo_node);
  executor.add_node(service_node);
  std::thread spinner([&executor]() { executor.spin(); });

  auto bean_move_group = makeMoveGroup(bean_node, "bean", "robot_bean");
  auto nemo_move_group = makeMoveGroup(nemo_node, "nemo", "robot_nemo");

  // One callback group per robot, both MutuallyExclusive: a robot's own
  // go_home/select_pose/move_to_pose callbacks still serialize against each
  // other (calling plan()/execute() on the same MoveGroupInterface from two
  // threads at once isn't safe), but bean's group and nemo's group are
  // independent, so the MultiThreadedExecutor can run a bean callback and a
  // nemo callback at the same time instead of queuing them behind each
  // other on the shared service_node's default callback group.
  auto bean_cb_group = service_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto nemo_cb_group = service_node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto bean_home_srv = service_node->create_service<std_srvs::srv::Trigger>(
    "/bean/go_home",
    [&bean_move_group, logger](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::string error;
      bool ok = goNamedHome(*bean_move_group, logger, "Bean home_pose", error);
      response->success = ok;
      response->message = ok ? "Bean moved to home_pose" : "Bean home_pose failed: " + error;
    },
    rclcpp::ServicesQoS(), bean_cb_group);

  auto nemo_home_srv = service_node->create_service<std_srvs::srv::Trigger>(
    "/nemo/go_home",
    [&nemo_move_group, logger](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      std::string error;
      bool ok = goNamedHome(*nemo_move_group, logger, "Nemo home_pose", error);
      response->success = ok;
      response->message = ok ? "Nemo moved to home_pose" : "Nemo home_pose failed: " + error;
    },
    rclcpp::ServicesQoS(), nemo_cb_group);

  auto bean_select_pose_srv = service_node->create_service<task_coordinator::srv::SelectPose>(
    "/bean/select_pose",
    [&bean_move_group, logger](
      const std::shared_ptr<task_coordinator::srv::SelectPose::Request> request,
      std::shared_ptr<task_coordinator::srv::SelectPose::Response> response)
    {
      std::string error;
      bool ok = selectNamedPose(
        *bean_move_group, logger, "Bean " + request->pose_name, kBeanPoses,
        request->pose_name, error);
      response->success = ok;
      response->message = ok ? "Bean moved to " + request->pose_name : error;
    },
    rclcpp::ServicesQoS(), bean_cb_group);

  auto nemo_select_pose_srv = service_node->create_service<task_coordinator::srv::SelectPose>(
    "/nemo/select_pose",
    [&nemo_move_group, logger](
      const std::shared_ptr<task_coordinator::srv::SelectPose::Request> request,
      std::shared_ptr<task_coordinator::srv::SelectPose::Response> response)
    {
      std::string error;
      bool ok = selectNamedPose(
        *nemo_move_group, logger, "Nemo " + request->pose_name, kNemoPoses,
        request->pose_name, error);
      response->success = ok;
      response->message = ok ? "Nemo moved to " + request->pose_name : error;
    },
    rclcpp::ServicesQoS(), nemo_cb_group);

  auto bean_move_to_pose_srv = service_node->create_service<task_coordinator::srv::MoveToPose>(
    "/bean/move_to_pose",
    [&bean_move_group, logger](
      const std::shared_ptr<task_coordinator::srv::MoveToPose::Request> request,
      std::shared_ptr<task_coordinator::srv::MoveToPose::Response> response)
    {
      std::string error;
      bool ok = goPoseTarget(*bean_move_group, logger, "Bean move_to_pose", request->target, error);
      response->success = ok;
      response->message = ok ? "Bean reached target pose" : "Bean move_to_pose failed: " + error;
    },
    rclcpp::ServicesQoS(), bean_cb_group);

  auto nemo_move_to_pose_srv = service_node->create_service<task_coordinator::srv::MoveToPose>(
    "/nemo/move_to_pose",
    [&nemo_move_group, logger](
      const std::shared_ptr<task_coordinator::srv::MoveToPose::Request> request,
      std::shared_ptr<task_coordinator::srv::MoveToPose::Response> response)
    {
      std::string error;
      bool ok = goPoseTarget(*nemo_move_group, logger, "Nemo move_to_pose", request->target, error);
      response->success = ok;
      response->message = ok ? "Nemo reached target pose" : "Nemo move_to_pose failed: " + error;
    },
    rclcpp::ServicesQoS(), nemo_cb_group);

  RCLCPP_INFO(
    logger,
    "Services ready: /bean/go_home, /nemo/go_home, /bean/select_pose, /nemo/select_pose, "
    "/bean/move_to_pose, /nemo/move_to_pose");

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
