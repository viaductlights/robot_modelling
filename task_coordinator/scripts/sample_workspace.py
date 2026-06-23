#!/usr/bin/env python3
"""Monte Carlo end-effector workspace sampler.

Diagnostic tool, not a maintained node - run directly with python3 after
sourcing install/setup.bash:

    python3 task_coordinator/scripts/sample_workspace.py

Samples random joint configurations within each robot's URDF joint limits,
calls that robot's compute_fk service for each one, and reports the
resulting position bounding box. Needs move_group (or hmi_motion_server)
running for the compute_fk service - no Gazebo physics/execution required,
this is pure kinematics.
"""

import random
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK

NUM_SAMPLES = 5000

ROBOTS = [
    {
        "name": "bean",
        "urdf_path": get_package_share_directory("bean3_description") + "/urdf/bean2.urdf",
        "tip_link": "link_six",
        "fk_service": "/bean/compute_fk",
    },
    {
        "name": "nemo",
        "urdf_path": get_package_share_directory("nemo1_description") + "/urdf/nemo1.urdf",
        "tip_link": "EE",
        "fk_service": "/nemo/compute_fk",
    },
]


def joint_limits(urdf_path: str) -> list:
    """Return [(name, lower, upper), ...] for every joint with a <limit> tag."""
    root = ET.parse(urdf_path).getroot()
    limits = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None:
            limits.append((joint.get("name"), float(limit.get("lower")), float(limit.get("upper"))))
    return limits


def sample_bounding_box(node: Node, robot: dict, limits: list) -> tuple:
    client = node.create_client(GetPositionFK, robot["fk_service"])
    node.get_logger().info(f"waiting for {robot['fk_service']}...")
    client.wait_for_service()

    names = [name for name, _, _ in limits]
    mins = [float("inf")] * 3
    maxs = [float("-inf")] * 3

    for i in range(NUM_SAMPLES):
        positions = [random.uniform(lower, upper) for _, lower, upper in limits]

        request = GetPositionFK.Request()
        request.header.frame_id = ""
        request.fk_link_names = [robot["tip_link"]]
        request.robot_state.joint_state = JointState(name=names, position=positions)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        response = future.result()
        if not response.pose_stamped:
            continue
        p = response.pose_stamped[0].pose.position
        for axis, value in enumerate((p.x, p.y, p.z)):
            mins[axis] = min(mins[axis], value)
            maxs[axis] = max(maxs[axis], value)

        if (i + 1) % 1000 == 0:
            node.get_logger().info(f"{robot['name']}: {i + 1}/{NUM_SAMPLES} samples")

    return tuple(mins), tuple(maxs)


def main():
    rclpy.init()
    node = Node("sample_workspace")

    for robot in ROBOTS:
        limits = joint_limits(robot["urdf_path"])
        mins, maxs = sample_bounding_box(node, robot, limits)
        print(f"\n{robot['name']} reachable bounding box (world frame, {NUM_SAMPLES} samples):")
        print(f"  x: [{mins[0]:.3f}, {maxs[0]:.3f}]")
        print(f"  y: [{mins[1]:.3f}, {maxs[1]:.3f}]")
        print(f"  z: [{mins[2]:.3f}, {maxs[2]:.3f}]")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
