#!/usr/bin/env python3
"""
Multi-Robot Control GUI
tkinter + rclpy (ROS2 Jazzy)

  - rclpy spins in a background thread, tkinter owns the main thread
  - "Home" goes through /<name>/go_home (Trigger); picking a pose from the
    first dropdown goes through /<name>/select_pose (named joint target);
    picking a pose from the second dropdown goes through /<name>/move_to_pose
    (arbitrary Cartesian target, MoveIt solves IK) - all three are
    task_coordinator/hmi_motion_server services, MoveIt-planned and
    collision-checked
  - Controls are only enabled once their service is available (backend ready)
  - EVERY rclpy call happens on the executor thread (marshalled via a queue)
  - home_pose comes from the SRDF (single source of truth); the other poses
    are hardcoded server-side (see hmi_motion_server.cpp) - the GUI only
    sends the chosen name (or, for move_to_pose, a hardcoded test Pose)
"""

import queue
import threading
from functools import partial

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
from task_coordinator.srv import SelectPose, MoveToPose


def _pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


# ---- Configuration --------------------------------------------------------
# "poses" must match the names hmi_motion_server.cpp knows about
# (kBeanPoses/kNemoPoses). "test_poses" are hardcoded Cartesian targets for
# the move_to_pose/IK feature - derived from the forward kinematics of an
# already-verified-reachable joint pose (pose_1), via
# `ros2 service call /<name>/compute_fk ...`, not guessed XYZ values, so
# they're known to be reachable. Values are in MoveIt's planning frame
# ("world" here - this sim places robots at ISS-scale coordinates, tens of
# meters from the world origin, not in each robot's own base_link frame).
ROBOT_CONFIG = [
    {
        "name": "nemo",
        "joint_states_topic": "/nemo/joint_states",
        "poses": ["pose_1"],
        "test_poses": {
            "Test Pose 1": _pose(
                90.2878776918658, -0.4593858596756446, -33.35982108426537,
                0.8808892143064816, 0.005274580505267679,
                0.46589638463297334, -0.08334824356234283),
        },
    },
    {
        "name": "bean",
        "joint_states_topic": "/bean/joint_states",
        "poses": ["pose_1", "pose_2"],
        "test_poses": {
            "Test Pose 1": _pose(
                94.7207851580837, -6.159334235737438, -33.87256215907665,
                -0.13143097974709886, 0.7688423246950394,
                -0.21001137463211458, 0.5894935112835216),
        },
    },
]
STALE_AFTER_SEC = 1.0   # no new JointState message for longer than this -> "offline"
ROS_TICK_SEC = 0.1      # ROS timer: refresh readiness + drain the queue
# --------------------------------------------------------------------------


class RobotHandle:
    """Encapsulates everything for ONE robot. rclpy objects are only
    touched by the ROS thread; the GUI only reads cached attributes."""

    def __init__(
            self, node: Node, name: str, joint_states_topic: str,
            poses: list, test_poses: dict):
        self.node = node
        self.name = name
        self.poses = poses
        self.test_poses = test_poses

        self.latest_state = None
        self.last_stamp = 0.0

        self._planner_ready = False              # ROS thread writes, GUI thread reads
        self.last_home_result = None              # (success|None, message) | None
        self.last_select_pose_result = None       # (success|None, message) | None
        self.last_move_to_pose_result = None      # (success|None, message) | None

        self.sub = node.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, 10)
        self.home_client = node.create_client(Trigger, f"/{name}/go_home")
        self.select_pose_client = node.create_client(SelectPose, f"/{name}/select_pose")
        self.move_to_pose_client = node.create_client(MoveToPose, f"/{name}/move_to_pose")

    # ---- ROS thread --------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        self.latest_state = msg
        self.last_stamp = self.node.get_clock().now().nanoseconds * 1e-9

    def refresh_readiness(self):
        self._planner_ready = (
            self.home_client.service_is_ready()
            and self.select_pose_client.service_is_ready()
            and self.move_to_pose_client.service_is_ready())

    # ---- GUI thread (only reads cached values) -----------------------------
    def is_online(self) -> bool:
        if self.latest_state is None:
            return False
        now = self.node.get_clock().now().nanoseconds * 1e-9
        return (now - self.last_stamp) < STALE_AFTER_SEC

    def planner_ready(self) -> bool:
        return self._planner_ready

    def print_state(self):
        if self.latest_state is None:
            self.node.get_logger().info(f"[{self.name}] no JointState received yet")
            return
        pairs = ", ".join(
            f"{n}={p:.3f}"
            for n, p in zip(self.latest_state.name, self.latest_state.position))
        self.node.get_logger().info(f"[{self.name}] {pairs}")


class MultiRobotNode(Node):
    """Holds all RobotHandles. A ROS timer refreshes readiness and
    drains the commands queued up by the GUI."""

    def __init__(self):
        super().__init__("hmi")
        self.robots = {
            cfg["name"]: RobotHandle(
                self, cfg["name"], cfg["joint_states_topic"],
                cfg["poses"], cfg["test_poses"])
            for cfg in ROBOT_CONFIG
        }
        self._cmd_queue = queue.Queue()
        self.create_timer(ROS_TICK_SEC, self._ros_tick)
        self.get_logger().info(f"GUI node started, robots: {list(self.robots)}")

    # ---- called from the GUI (queue only, thread-safe) --------------------
    def request_go_home(self, name: str):
        self._cmd_queue.put(("home", name, None))

    def request_select_pose(self, name: str, pose_name: str):
        self._cmd_queue.put(("select_pose", name, pose_name))

    def request_move_to_pose(self, name: str, test_pose_name: str):
        self._cmd_queue.put(("move_to_pose", name, test_pose_name))

    # ---- ROS thread --------------------------------------------------------
    def _ros_tick(self):
        for handle in self.robots.values():
            handle.refresh_readiness()
        self._drain_commands()

    def _drain_commands(self):
        while True:
            try:
                action, name, choice = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            handle = self.robots.get(name)
            if handle is None:
                continue
            if not handle.planner_ready():
                self._set_result(handle, action, (False, "Planner not ready"))
                continue
            self._set_result(handle, action, (None, "running..."))
            if action == "home":
                client, request = handle.home_client, Trigger.Request()
            elif action == "select_pose":
                client = handle.select_pose_client
                request = SelectPose.Request(pose_name=choice)
            else:  # move_to_pose
                client = handle.move_to_pose_client
                request = MoveToPose.Request(target=handle.test_poses[choice])
            future = client.call_async(request)
            future.add_done_callback(partial(self._on_command_done, handle, action))

    def _set_result(self, handle: RobotHandle, action: str, result):
        if action == "home":
            handle.last_home_result = result
        elif action == "select_pose":
            handle.last_select_pose_result = result
        else:
            handle.last_move_to_pose_result = result

    def _on_command_done(self, handle: RobotHandle, action: str, future):
        try:
            res = future.result()
            self._set_result(handle, action, (res.success, res.message))
            self.get_logger().info(
                f"[{handle.name}] {action} -> {res.success}: {res.message}")
        except Exception as exc:  # pragma: no cover
            self._set_result(handle, action, (False, f"Service error: {exc}"))


class RobotPanel(ttk.LabelFrame):
    def __init__(self, parent, handle: RobotHandle):
        super().__init__(parent, text=handle.name, padding=10)
        self.handle = handle

        self.status = tk.Label(self, text="\u25cf offline", fg="#c0392b",
                               font=("TkDefaultFont", 10, "bold"))
        self.status.pack(anchor="w")

        self.planner_status = tk.Label(self, text="Planner: waiting...",
                                       fg="#c0392b", font=("TkDefaultFont", 9))
        self.planner_status.pack(anchor="w")

        self.joints = tk.Text(self, height=8, width=40, state="disabled",
                              font=("TkFixedFont", 9))
        self.joints.pack(pady=6)

        btns = ttk.Frame(self)
        btns.pack(anchor="w")
        ttk.Button(btns, text="Print State",
                   command=handle.print_state).pack(side="left", padx=2)
        self.home_btn = ttk.Button(
            btns, text="Home",
            command=lambda: handle.node.request_go_home(handle.name),
            state="disabled")
        self.home_btn.pack(side="left", padx=2)

        pose_row = ttk.Frame(self)
        pose_row.pack(anchor="w", pady=(4, 0))
        self.pose_combo = ttk.Combobox(
            pose_row, values=handle.poses, state="disabled", width=14)
        self.pose_combo.current(0)
        self.pose_combo.pack(side="left", padx=2)
        self.select_pose_btn = ttk.Button(
            pose_row, text="Go",
            command=lambda: handle.node.request_select_pose(
                handle.name, self.pose_combo.get()),
            state="disabled")
        self.select_pose_btn.pack(side="left", padx=2)

        test_pose_row = ttk.Frame(self)
        test_pose_row.pack(anchor="w", pady=(4, 0))
        self.test_pose_combo = ttk.Combobox(
            test_pose_row, values=list(handle.test_poses.keys()),
            state="disabled", width=14)
        self.test_pose_combo.current(0)
        self.test_pose_combo.pack(side="left", padx=2)
        self.move_to_pose_btn = ttk.Button(
            test_pose_row, text="Move To Pose",
            command=lambda: handle.node.request_move_to_pose(
                handle.name, self.test_pose_combo.get()),
            state="disabled")
        self.move_to_pose_btn.pack(side="left", padx=2)

        self.result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.result.pack(anchor="w")
        self.select_pose_result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.select_pose_result.pack(anchor="w")
        self.move_to_pose_result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.move_to_pose_result.pack(anchor="w")

    def refresh(self):
        online = self.handle.is_online()
        self.status.config(
            text="\u25cf online" if online else "\u25cf offline",
            fg="#27ae60" if online else "#c0392b")

        ready = self.handle.planner_ready()
        self.planner_status.config(
            text="Planner: ready" if ready else "Planner: waiting for backend services...",
            fg="#27ae60" if ready else "#c0392b")
        self.home_btn.config(state="normal" if ready else "disabled")
        self.pose_combo.config(state="readonly" if ready else "disabled")
        self.select_pose_btn.config(state="normal" if ready else "disabled")
        self.test_pose_combo.config(state="readonly" if ready else "disabled")
        self.move_to_pose_btn.config(state="normal" if ready else "disabled")

        self._update_result_label(self.result, "Home", self.handle.last_home_result)
        self._update_result_label(
            self.select_pose_result, "Pose", self.handle.last_select_pose_result)
        self._update_result_label(
            self.move_to_pose_result, "Move To Pose", self.handle.last_move_to_pose_result)

        self.joints.config(state="normal")
        self.joints.delete("1.0", "end")
        st = self.handle.latest_state
        if st is None:
            self.joints.insert("end", "waiting for JointState...")
        else:
            for n, p in zip(st.name, st.position):
                self.joints.insert("end", f"{n:<24}{p:+.4f}\n")
        self.joints.config(state="disabled")

    @staticmethod
    def _update_result_label(label: tk.Label, prefix: str, result):
        if result is None:
            label.config(text="")
            return
        ok, msg = result
        color = "#555" if ok is None else ("#27ae60" if ok else "#c0392b")
        label.config(text=f"{prefix}: {msg}", fg=color)


class App:
    def __init__(self, node: MultiRobotNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("nemo1 / bean2 - Control")

        container = ttk.Frame(self.root, padding=10)
        container.pack(fill="both", expand=True)

        self.panels = []
        for col, handle in enumerate(node.robots.values()):
            panel = RobotPanel(container, handle)
            panel.grid(row=0, column=col, padx=8, pady=4, sticky="n")
            self.panels.append(panel)

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._refresh_loop()

    def _refresh_loop(self):
        for panel in self.panels:
            panel.refresh()
        self.root.after(100, self._refresh_loop)

    def _on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = MultiRobotNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = App(node)
    app.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()