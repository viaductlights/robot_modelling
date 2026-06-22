#include "task_coordinator/arm_setup.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

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
  const std::string & label)
{
  group.setNamedTarget("home_pose");
  return planAndExecute(group, logger, label);
}

bool goJointTargets(
  MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label,
  const std::vector<double> & joint_targets)
{
  if (!group.setJointValueTarget(joint_targets)) {
    RCLCPP_ERROR(logger, "%s: joint targets out of bounds or invalid size", label.c_str());
    return false;
  }
  return planAndExecute(group, logger, label);
}

bool runJointSequence(
  MoveGroupInterface & group,
  const rclcpp::Logger & logger,
  const std::string & label,
  const std::vector<std::vector<double>> & sequence)
{
  for (size_t i = 0; i < sequence.size(); ++i) {
    if (!goJointTargets(group, logger, label + " step " + std::to_string(i + 1), sequence[i])) {
      return false;
    }
  }
  return true;
}

// Demo task sequences, carried over from sim_task.cpp's hardcoded joint
// targets (placeholders - known to be inside each robot's workspace).
const std::vector<std::vector<double>> kBeanTaskSequence = {
  {-0.174, 0.384, 1.309, -1.326, 1.501, -2.25},
  {-0.506, 0.436, -0.523, -0.733, -0.576, -1.826},
};
const std::vector<std::vector<double>> kNemoTaskSequence = {
  {-0.907, -0.192, -0.174, -2.077, 0.716, 0.628, -0.506},
};
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
  // go_home/run_task callbacks still serialize against each other (calling
  // plan()/execute() on the same MoveGroupInterface from two threads at
  // once isn't safe), but bean's group and nemo's group are independent,
  // so the MultiThreadedExecutor can run a bean callback and a nemo
  // callback at the same time instead of queuing them behind each other
  // on the shared service_node's default callback group.
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
      bool ok = goNamedHome(*bean_move_group, logger, "Bean home_pose");
      response->success = ok;
      response->message = ok ? "Bean moved to home_pose" : "Bean home_pose failed";
    },
    rclcpp::ServicesQoS(), bean_cb_group);

  auto nemo_home_srv = service_node->create_service<std_srvs::srv::Trigger>(
    "/nemo/go_home",
    [&nemo_move_group, logger](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      bool ok = goNamedHome(*nemo_move_group, logger, "Nemo home_pose");
      response->success = ok;
      response->message = ok ? "Nemo moved to home_pose" : "Nemo home_pose failed";
    },
    rclcpp::ServicesQoS(), nemo_cb_group);

  auto bean_task_srv = service_node->create_service<std_srvs::srv::Trigger>(
    "/bean/run_task",
    [&bean_move_group, logger](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      bool ok = runJointSequence(*bean_move_group, logger, "Bean task", kBeanTaskSequence);
      response->success = ok;
      response->message = ok ? "Bean task sequence done" : "Bean task sequence failed";
    },
    rclcpp::ServicesQoS(), bean_cb_group);

  auto nemo_task_srv = service_node->create_service<std_srvs::srv::Trigger>(
    "/nemo/run_task",
    [&nemo_move_group, logger](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      bool ok = runJointSequence(*nemo_move_group, logger, "Nemo task", kNemoTaskSequence);
      response->success = ok;
      response->message = ok ? "Nemo task sequence done" : "Nemo task sequence failed";
    },
    rclcpp::ServicesQoS(), nemo_cb_group);

  RCLCPP_INFO(
    logger,
    "Services ready: /bean/go_home, /nemo/go_home, /bean/run_task, /nemo/run_task");

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
