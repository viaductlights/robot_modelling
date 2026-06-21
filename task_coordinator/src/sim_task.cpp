#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <sstream>

// Most rudimentary possible: one namespaced node per robot, each owning
// a MoveGroupInterface targeting that robot's move_group, three blocking
// moves in sequence: bean -> pose1, bean -> pose2, nemo -> pose3.
//
// NOTE: pose1/pose2/pose3 below are placeholders. Pick values you know
// are inside each robot's reachable workspace - if planning fails,
// each robot's SRDF already defines a "home_pose" group_state you can
// fall back to with setNamedTarget("home_pose") to confirm the
// MoveGroupInterface connection itself is working before chasing
// reachability.

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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // One namespaced node per robot - MoveGroupInterface's internal
  // RDFLoader looks for "robot_description"/"robot_description_semantic"
  // as plain relative names resolved against whatever node it's given.
  // A genuinely namespaced node resolves these (and everything else)
  // via normal relative-name resolution, no special-casing needed.
  auto bean_node = rclcpp::Node::make_shared("sim_task_bean", "bean");
  auto nemo_node = rclcpp::Node::make_shared("sim_task_nemo", "nemo");
  auto logger = rclcpp::get_logger("sim_task");

  // robot_description (the URDF) is covered by robot_state_publisher's
  // fallback topic at /bean/robot_description and /nemo/robot_description.
  // robot_description_semantic (the SRDF) has no equivalent fallback
  // publisher anywhere in the stack - it only ever exists as a parameter,
  // normally one move_group already has via MoveItConfigsBuilder in the
  // bringup launch file. sim_task is a separate process started with
  // `ros2 run`, so it has to load and declare it directly.
  bean_node->declare_parameter(
      "robot_description_semantic",
      readFile(ament_index_cpp::get_package_share_directory("bean2_moveit") + "/config/bean2.srdf"));
  nemo_node->declare_parameter(
      "robot_description_semantic",
      readFile(ament_index_cpp::get_package_share_directory("nemo1_moveit") + "/config/nemo1.srdf"));

  // robot_description_kinematics is NOT a single string like the two
  // above - it has to be actual nested ROS parameters
  // (robot_description_kinematics.<group>.<key>), since RobotModelLoader
  // looks up each key individually rather than parsing a blob of YAML
  // text. Both kinematics.yaml files here are simple/identical, so these
  // are declared directly; update by hand if kinematics.yaml ever changes.
  for (auto& node : {bean_node, nemo_node}) {
    node->declare_parameter("robot_description_kinematics.robot.kinematics_solver",
                             std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
    node->declare_parameter("robot_description_kinematics.robot.kinematics_solver_search_resolution", 0.005);
    node->declare_parameter("robot_description_kinematics.robot.kinematics_solver_timeout", 0.005);
  }

  // MoveGroupInterface needs the nodes spinning concurrently (current
  // state monitor, action client callbacks) - without this, plan()/
  // execute() will just hang.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bean_node);
  executor.add_node(nemo_node);
  std::thread spinner([&executor]() { executor.spin(); });

  using moveit::planning_interface::MoveGroupInterface;

  // group_name "robot" matches both bean2.srdf and nemo1.srdf.
  //
  // Both the namespaced node AND Options::move_group_namespace_ are
  // needed together, for two different reasons: robot_description/
  // robot_description_semantic's topic fallbacks genuinely use ordinary
  // relative-name resolution against the node (so the node itself has
  // to be namespaced "bean"/"nemo"), but the move_action action client
  // is built separately from Options::move_group_namespace_ as an
  // ABSOLUTE path - when that's left empty, it resolves to plain
  // /move_action regardless of the node's own namespace, not /bean/
  // move_action. Confirmed via gdb: the action client object existed
  // and was correctly waiting, just on the wrong (unnamespaced) name.
  MoveGroupInterface::Options bean_options("robot","robot_description", "/bean");
  MoveGroupInterface::Options nemo_options("robot", "robot_description", "/nemo");
  MoveGroupInterface bean_move_group(bean_node, bean_options);
  MoveGroupInterface nemo_move_group(nemo_node, nemo_options);

  // pilz_industrial_motion_planner is the default pipeline in this
  // setup (no ompl_planning.yaml is present in either robot's config -
  // ompl is still available via moveit_configs_utils' bundled generic
  // default, just not selected by default). Matches manually switching
  // the pipeline dropdown in RViz's MotionPlanning panel from pilz to
  // ompl - no specific planner_id set, so each pipeline uses its own
  // default planner.
  bean_move_group.setPlanningPipelineId("ompl");
  nemo_move_group.setPlanningPipelineId("ompl");

  auto plan_and_execute_joint = [&logger](MoveGroupInterface& group, const std::vector<double>& joint_targets,
                                     const std::string& label) {
//    group.setPoseTarget(target);

    if (!group.setJointValueTarget(joint_targets)) {
	RCLCPP_ERROR(logger, "%s: Joint targets out of bounds or invalid size", label.c_str());
        return false;
    }

    MoveGroupInterface::Plan plan;
 
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
  };

/*  geometry_msgs::msg::Pose pose1;
  pose1.position.x = 1.74; 
  pose1.position.y = -5.695;
  pose1.position.z = -1.08;
  pose1.orientation.x = 0.0;
  pose1.orientation.y = 0.707;
  pose1.orientation.z = -0.0;
  pose1.orientation.w = 0.707;
  plan_and_execute(bean_move_group, pose1, "bean -> pose1");*/

  std::vector<double> my_joint_goals = {0.22, 0.0, 0.0, 0.0, 0.0, 0.0};
  bool joint_success = plan_and_execute_joint(bean_move_group, my_joint_goals, "Bean moving joints");
  RCLCPP_INFO(logger, "status: %d", joint_success);

  rclcpp::shutdown();
  spinner.join();
  return 0;
}

