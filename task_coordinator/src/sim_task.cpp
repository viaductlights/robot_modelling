#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <std_msgs/msg/empty.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace
{
std::string readFile(const std::string& path)
{
  std::ifstream file(path);
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

class TaskCoordinator
{
public:
  using MGI = moveit::planning_interface::MoveGroupInterface;

  TaskCoordinator(rclcpp::Node::SharedPtr bean_node, rclcpp::Node::SharedPtr nemo_node)
  : bean_node_(bean_node),
    nemo_node_(nemo_node),
    logger_(rclcpp::get_logger("task_coordinator")),
    bean_group_(bean_node_, MGI::Options("robot_bean", "bean_robot_description", "/bean")),
    nemo_group_(nemo_node_, MGI::Options("robot_nemo", "nemo_robot_description", "/nemo")),
    bean_attach_pub_(bean_node_->create_publisher<std_msgs::msg::Empty>("/bean/attach", 1)),
    bean_detach_pub_(bean_node_->create_publisher<std_msgs::msg::Empty>("/bean/detach", 1))
//    nemo_attach_pub_(nemo_node_->create_publisher<std_msgs::msg::Empty>("/nemo/attach", 1)),
//    nemo_detach_pub_(nemo_node_->create_pubsliher<std_msgs::msg::Empty>("/nemo/detach", 1)),
  {
    bean_group_.setPlanningPipelineId("ompl");
    nemo_group_.setPlanningPipelineId("ompl");
    bean_group_.setMaxVelocityScalingFactor(0.6);
    bean_group_.setMaxAccelerationScalingFactor(0.2);
    nemo_group_.setMaxVelocityScalingFactor(0.8);
    nemo_group_.setMaxAccelerationScalingFactor(0.3);
  }

  void run(){

    // DetachableJoint plugin has no start state. need ensure detachment to prevent microgravity + physics engine drift effects
    detachCapsuleFromBean();
      
    // bean to goal1, then attach capsule
    RCLCPP_INFO(logger_, "moving Bean to attachment position");
    bool joint_success1 = moveTo(bean_group_,
               {0.227, 0.419, 1.344, 2.496, -1.309, -0.750},
               "bean -> goal1");
    RCLCPP_INFO(logger_, "status of first pose: %d", joint_success1);

    if (!joint_success1) {
      RCLCPP_ERROR(logger_, "Failed to reach attachment position. Aborting.");
      return;
    }

    RCLCPP_INFO(logger_, "attaching capsule to Bean");
    attachCapsuleToBean();

        std::this_thread::sleep_for(1s);
   // bean to goal2 carrying capsule
    bool joint_success2 = moveTo(bean_group_,
           {-0.87, 1.45, 1.065, 1.326, -5.39, 2.28},
           "bean -> goal2");
    RCLCPP_INFO(logger_, "status of second pose: %d", joint_success2);

    // nemo to goal3, then transfer capsule from bean to nemo
    bool joint_success3 = moveTo(nemo_group_,
               {1.29, 1.45, -0.52, 0.87, 0.14, 1.82, -1.71},
               "nemo -> goal3");
    RCLCPP_INFO(logger_, "status of third pose: %d", joint_success3);
    //{
     // attachCapsuleToNemo();
      //detachCapsuleFromBean();
    //}
  }

private:
  bool moveTo(MGI& group, const std::vector<double>& joints, const std::string& label)
  {
    if (!group.setJointValueTarget(joints)) {
      RCLCPP_ERROR(logger_, "%s: joint targets out of bounds or invalid size", label.c_str());
      return false;
    }
    MGI::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "%s: planning failed", label.c_str());
      return false;
    }
    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(logger_, "%s: execution failed", label.c_str());
      return false;
    }
    RCLCPP_INFO(logger_, "%s: done", label.c_str());
    return true;
  }

  void attachCapsuleToBean()
  {
    RCLCPP_INFO(logger_, "publishing attach capsule to bean");
    bean_attach_pub_->publish(std_msgs::msg::Empty{});
    RCLCPP_INFO(logger_, "Attach message published");
  }

  void detachCapsuleFromBean()
  {
    RCLCPP_INFO(logger_, "publishing detaching capsule from bean");
    bean_detach_pub_->publish(std_msgs::msg::Empty{});
    RCLCPP_INFO(logger_, "detach  message published");
  }

  /*void attachCapsuleToNemo()
  {
    RCLCPP_INFO(logger_, "publishing attaching capsule to nemo");
    nemo_attach_pub_->publish(std_msgs::msg::Empty{});
  }*/

  // Declared in initialization order - do not reorder without updating the member initializer list accordingly!!
  rclcpp::Node::SharedPtr bean_node_;
  rclcpp::Node::SharedPtr nemo_node_;
  rclcpp::Logger logger_;
  MGI bean_group_;
  MGI nemo_group_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr bean_attach_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr bean_detach_pub_;
  //rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr nemo_attach_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto bean_node = rclcpp::Node::make_shared("sim_task_bean", "bean");
  auto nemo_node = rclcpp::Node::make_shared("sim_task_nemo", "nemo");

  // All parameter declarations before TaskCoordinator construction -
  // MoveGroupInterface reads these during its own constructor.
  bean_node->declare_parameter(
      "bean_robot_description",
      readFile(ament_index_cpp::get_package_share_directory("bean3_description") + "/urdf/bean2.urdf"));
  nemo_node->declare_parameter(
      "nemo_robot_description",
      readFile(ament_index_cpp::get_package_share_directory("nemo1_description") + "/urdf/nemo1.urdf"));

  bean_node->declare_parameter(
      "bean_robot_description_semantic",
      readFile(ament_index_cpp::get_package_share_directory("bean2_moveit") + "/config/bean2.srdf"));
  nemo_node->declare_parameter(
      "nemo_robot_description_semantic",
      readFile(ament_index_cpp::get_package_share_directory("nemo1_moveit") + "/config/nemo1.srdf"));

  // Kinematics: nested ROS 2 parameters matching each robot's group name.  Declared on both nodes since RobotModelLoader looks on whatever node
  // Adjust values if kinematics.yaml ever changes!!
  for (auto& node : {bean_node, nemo_node}) {
    node->declare_parameter(
        "bean_robot_description_kinematics.robot_bean.kinematics_solver",
        std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
    node->declare_parameter(
        "bean_robot_description_kinematics.robot_bean.kinematics_solver_search_resolution", 0.005);
    node->declare_parameter(
        "bean_robot_description_kinematics.robot_bean.kinematics_solver_timeout", 0.005);
    node->declare_parameter(
        "nemo_robot_description_kinematics.robot_nemo.kinematics_solver",
        std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
    node->declare_parameter(
        "nemo_robot_description_kinematics.robot_nemo.kinematics_solver_search_resolution", 0.005);
    node->declare_parameter(
        "nemo_robot_description_kinematics.robot_nemo.kinematics_solver_timeout", 0.005);
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bean_node);
  executor.add_node(nemo_node);
  std::thread spinner([&executor]() { executor.spin(); });

  TaskCoordinator coordinator(bean_node, nemo_node);
  coordinator.run();

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
