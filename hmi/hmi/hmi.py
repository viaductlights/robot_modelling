#!/usr/bin/env python3
"""
Multi-Robot Control GUI
tkinter + rclpy (ROS2 Jazzy)

  - rclpy spins in a background thread, tkinter owns the main thread
  - "Home" goes through /<name>/go_home (Trigger); picking a pose from the
    first dropdown goes through /<name>/select_pose (named joint target);
    picking a pose from the second dropdown, or entering X/Y/Z/Roll/Pitch/Yaw
    in the "Custom Pose (IK)" panel, goes through /<name>/move_to_pose
    (arbitrary Cartesian target, MoveIt solves IK) - all are
    task_coordinator/hmi_motion_server services, MoveIt-planned and
    collision-checked
  - Controls are only enabled once their service is available (backend ready)
  - EVERY rclpy call happens on the executor thread (marshalled via a queue)
  - home_pose comes from the SRDF (single source of truth); the other poses
    are hardcoded server-side (see hmi_motion_server.cpp) - the GUI only
    sends the chosen name (or a Pose, for move_to_pose/move_to_custom_pose)
  - the "reachable hint" bounding box shown in the Custom Pose panel comes
    from task_coordinator/scripts/sample_workspace.py (a Monte Carlo FK
    sweep) - it's just a hint, not enforced; MoveIt's planner/IK is the
    real reachability check
"""

import math
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
from task_coordinator.srv import SelectPose, MoveToPose, JogStep


def _pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def _euler_to_quaternion(roll, pitch, yaw):
    """roll/pitch/yaw in radians, intrinsic ZYX (REP-103) convention."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _quaternion_to_euler(qx, qy, qz, qw):
    """Inverse of _euler_to_quaternion - only used to pre-fill the GUI's
    custom-pose fields from an existing Pose, so it just needs to be the
    self-consistent inverse of the function above, not match any external
    convention bit-for-bit."""
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


# ---- Configuration --------------------------------------------------------
# "poses" must match the names hmi_motion_server.cpp knows about
# (kBeanPoses/kNemoPoses). "test_poses" are hardcoded Cartesian targets for
# the move_to_pose/IK feature - derived from the forward kinematics of an
# already-verified-reachable joint pose (pose_1), via
# `ros2 service call /<name>/compute_fk ...`, not guessed XYZ values, so
# they're known to be reachable. Values are in MoveIt's planning frame
# ("world" here - this sim places robots at ISS-scale coordinates, tens of
# meters from the world origin, not in each robot's own base_link frame).
# "workspace_bounds" is the end-effector position bounding box from
# task_coordinator/scripts/sample_workspace.py (5000-sample Monte Carlo FK
# sweep over the URDF's joint limits), rounded outward to 1 decimal (floor
# the lower bound, ceil the upper bound, so rounding only widens the range,
# never shrinks it past what was actually sampled reachable). Used as the
# custom-pose sliders' drag range - the real reachability check is still
# whether MoveIt's planner/IK succeeds.
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
        "workspace_bounds": {"x": (87.3, 92.4), "y": (-3.1, 1.9), "z": (-37.2, -32.0)},
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
        "workspace_bounds": {"x": (77.2, 107.7), "y": (-18.1, 11.3), "z": (-50.1, -19.9)},
    },
]
STALE_AFTER_SEC = 1.0   # no new JointState message for longer than this -> "offline"
ROS_TICK_SEC = 0.1      # ROS timer: refresh readiness + drain the queue
JOG_STEP_M_DEFAULT = 0.001  # default discrete Cartesian jog step, in meters
JOG_REPEAT_MS = 400     # hold-to-repeat interval for jog buttons
# --------------------------------------------------------------------------


class RobotHandle:
    """Encapsulates everything for ONE robot. rclpy objects are only
    touched by the ROS thread; the GUI only reads cached attributes."""

    def __init__(
            self, node: Node, name: str, joint_states_topic: str,
            poses: list, test_poses: dict, workspace_bounds: dict):
        self.node = node
        self.name = name
        self.poses = poses
        self.test_poses = test_poses
        self.workspace_bounds = workspace_bounds

        self.latest_state = None
        self.last_stamp = 0.0

        self._planner_ready = False              # ROS thread writes, GUI thread reads
        self.last_home_result = None              # (success|None, message) | None
        self.last_select_pose_result = None       # (success|None, message) | None
        self.last_move_to_pose_result = None      # (success|None, message) | None
        self.last_jog_result = None               # (success|None, message) | None
        self.jog_in_flight = False                # ROS thread writes, GUI thread reads

        self.sub = node.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, 10)
        self.home_client = node.create_client(Trigger, f"/{name}/go_home")
        self.select_pose_client = node.create_client(SelectPose, f"/{name}/select_pose")
        self.move_to_pose_client = node.create_client(MoveToPose, f"/{name}/move_to_pose")
        self.jog_client = node.create_client(JogStep, f"/{name}/jog")

    # ---- ROS thread --------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        self.latest_state = msg
        self.last_stamp = self.node.get_clock().now().nanoseconds * 1e-9

    def refresh_readiness(self):
        self._planner_ready = (
            self.home_client.service_is_ready()
            and self.select_pose_client.service_is_ready()
            and self.move_to_pose_client.service_is_ready()
            and self.jog_client.service_is_ready())

    # ---- GUI thread (only reads cached values) -----------------------------
    def is_online(self) -> bool:
        if self.latest_state is None:
            return False
        now = self.node.get_clock().now().nanoseconds * 1e-9
        return (now - self.last_stamp) < STALE_AFTER_SEC

    def planner_ready(self) -> bool:
        return self._planner_ready

    def is_jog_busy(self) -> bool:
        return self.jog_in_flight

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
                cfg["poses"], cfg["test_poses"], cfg["workspace_bounds"])
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

    def request_move_to_custom_pose(self, name: str, pose: Pose):
        self._cmd_queue.put(("move_to_custom_pose", name, pose))

    def request_jog(self, name: str, dx: float, dy: float, dz: float, in_tool_frame: bool):
        self._cmd_queue.put(("jog", name, (dx, dy, dz, in_tool_frame)))

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
            if action == "jog" and handle.jog_in_flight:
                continue
            self._set_result(handle, action, (None, "running..."))
            if action == "home":
                client, request = handle.home_client, Trigger.Request()
            elif action == "select_pose":
                client = handle.select_pose_client
                request = SelectPose.Request(pose_name=choice)
            elif action == "move_to_pose":
                client = handle.move_to_pose_client
                request = MoveToPose.Request(target=handle.test_poses[choice])
            elif action == "jog":
                client = handle.jog_client
                dx, dy, dz, in_tool_frame = choice
                request = JogStep.Request(dx=dx, dy=dy, dz=dz, in_tool_frame=in_tool_frame)
                handle.jog_in_flight = True
            else:  # move_to_custom_pose - choice is already a literal Pose
                client = handle.move_to_pose_client
                request = MoveToPose.Request(target=choice)
            future = client.call_async(request)
            future.add_done_callback(partial(self._on_command_done, handle, action))

    def _set_result(self, handle: RobotHandle, action: str, result):
        if action == "home":
            handle.last_home_result = result
        elif action == "select_pose":
            handle.last_select_pose_result = result
        elif action == "jog":
            handle.last_jog_result = result
        else:
            handle.last_move_to_pose_result = result

    def _on_command_done(self, handle: RobotHandle, action: str, future):
        if action == "jog":
            handle.jog_in_flight = False
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

        custom_pose_frame = ttk.LabelFrame(self, text="Custom Pose (IK)", padding=6)
        custom_pose_frame.pack(anchor="w", pady=(6, 0), fill="x")

        bounds = handle.workspace_bounds
        seed_pose = next(iter(handle.test_poses.values()))
        roll, pitch, yaw = _quaternion_to_euler(
            seed_pose.orientation.x, seed_pose.orientation.y,
            seed_pose.orientation.z, seed_pose.orientation.w)
        seed_values = [
            seed_pose.position.x, seed_pose.position.y, seed_pose.position.z,
            math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
        ]
        # Slider ranges: X/Y/Z come from sample_workspace.py's Monte Carlo FK
        # sweep (a position-only hint, not a guarantee - orientation affects
        # reachability too, see ROBOT_CONFIG comment). Roll/Pitch/Yaw just
        # get the full -180..180 range since we have no orientation-reachability
        # data. Sliders only bound the *drag* range - typing a number in the
        # entry still sends exactly what's typed, slider or not.
        field_specs = [
            ("X (m)", bounds["x"]), ("Y (m)", bounds["y"]), ("Z (m)", bounds["z"]),
            ("Roll (deg)", (-180.0, 180.0)), ("Pitch (deg)", (-180.0, 180.0)),
            ("Yaw (deg)", (-180.0, 180.0)),
        ]

        self.custom_pose_entries = {}
        self.custom_pose_scales = {}
        for col, ((label, (lo, hi)), value) in enumerate(zip(field_specs, seed_values)):
            tk.Label(custom_pose_frame, text=label, font=("TkDefaultFont", 8)).grid(
                row=0, column=col, sticky="w")

            entry = ttk.Entry(custom_pose_frame, width=8)
            entry.insert(0, f"{value:.3f}")
            entry.grid(row=2, column=col, padx=2)

            def on_slide(v, entry=entry):
                entry.delete(0, "end")
                entry.insert(0, f"{float(v):.3f}")

            scale = ttk.Scale(custom_pose_frame, from_=lo, to=hi, orient="horizontal",
                               length=100, command=on_slide)
            scale.set(value)
            scale.grid(row=1, column=col, padx=2, sticky="ew")

            def on_entry_commit(event, scale=scale, entry=entry, lo=lo, hi=hi):
                # Clamping (rather than rejecting) an out-of-range typed
                # value: scale.set() clamps to [lo, hi] and fires on_slide,
                # which rewrites the entry to match - so entry and slider
                # always agree, and a stray "1000" doesn't linger looking
                # like a sane number.
                try:
                    value = float(entry.get())
                except ValueError:
                    return
                scale.set(max(lo, min(hi, value)))

            entry.bind("<Return>", on_entry_commit)
            entry.bind("<FocusOut>", on_entry_commit)

            self.custom_pose_entries[label] = entry
            self.custom_pose_scales[label] = scale

        self.move_to_custom_pose_btn = ttk.Button(
            custom_pose_frame, text="Move To Pose",
            command=self._on_move_to_custom_pose, state="disabled")
        self.move_to_custom_pose_btn.grid(row=3, column=0, columnspan=6, pady=(4, 0), sticky="w")

        jog_frame = ttk.LabelFrame(self, text="Jog End Effector", padding=6)
        jog_frame.pack(anchor="w", pady=(6, 0), fill="x")

        tk.Label(jog_frame, text="step (m)", font=("TkDefaultFont", 8), fg="#555").grid(
            row=0, column=0, sticky="w")
        self.jog_step_var = tk.StringVar(value=f"{JOG_STEP_M_DEFAULT:.3f}")
        self.jog_step_entry = ttk.Entry(jog_frame, textvariable=self.jog_step_var, width=7)
        self.jog_step_entry.grid(row=0, column=1, sticky="w", padx=(3, 8))

        tk.Label(jog_frame, text="frame", font=("TkDefaultFont", 8), fg="#555").grid(
            row=0, column=2, sticky="w")
        self.jog_frame_var = tk.StringVar(value="tool")
        self.jog_frame_combo = ttk.Combobox(
            jog_frame, textvariable=self.jog_frame_var, values=["tool", "world"],
            state="readonly", width=6)
        self.jog_frame_combo.grid(row=0, column=3, sticky="w", padx=(3, 0))

        tk.Label(
            jog_frame, text="tool = end-effector frame, world = planning frame",
            font=("TkDefaultFont", 8), fg="#555").grid(
                row=1, column=0, columnspan=4, sticky="w", pady=(2, 3))

        jog_specs = [
            ("-X", (-1.0, 0.0, 0.0), 2, 0),
            ("+X", (+1.0, 0.0, 0.0), 2, 2),
            ("-Y", (0.0, -1.0, 0.0), 3, 0),
            ("+Y", (0.0, +1.0, 0.0), 3, 2),
            ("-Z", (0.0, 0.0, -1.0), 4, 0),
            ("+Z", (0.0, 0.0, +1.0), 4, 2),
        ]
        self.jog_buttons = []
        for text, direction, row, col in jog_specs:
            btn = ttk.Button(jog_frame, text=text, width=4, state="disabled")
            btn.grid(row=row, column=col, padx=2, pady=1)
            self._bind_jog_button(btn, direction)
            self.jog_buttons.append(btn)

        self.result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.result.pack(anchor="w")
        self.select_pose_result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.select_pose_result.pack(anchor="w")
        self.move_to_pose_result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.move_to_pose_result.pack(anchor="w")
        self.jog_result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.jog_result.pack(anchor="w")

    def _current_jog_step(self) -> float:
        try:
            step = float(self.jog_step_var.get())
        except ValueError:
            self.handle.last_jog_result = (False, "Invalid jog step")
            return JOG_STEP_M_DEFAULT
        if step <= 0.0:
            self.handle.last_jog_result = (False, "Jog step must be > 0")
            return JOG_STEP_M_DEFAULT
        return max(0.0001, min(0.05, step))

    def _bind_jog_button(self, btn: ttk.Button, direction):
        ux, uy, uz = direction
        state = {"after_id": None}

        def fire():
            step = self._current_jog_step()
            in_tool_frame = self.jog_frame_var.get() == "tool"
            self.handle.node.request_jog(
                self.handle.name, ux * step, uy * step, uz * step, in_tool_frame)

        def repeat():
            fire()
            state["after_id"] = btn.after(JOG_REPEAT_MS, repeat)

        def start(event=None):
            if not self.handle.planner_ready() or self.handle.is_jog_busy():
                return
            fire()
            state["after_id"] = btn.after(JOG_REPEAT_MS, repeat)

        def stop(event=None):
            if state["after_id"] is not None:
                btn.after_cancel(state["after_id"])
                state["after_id"] = None

        btn.bind("<ButtonPress-1>", start)
        btn.bind("<ButtonRelease-1>", stop)
        btn.bind("<Leave>", stop)

    def _on_move_to_custom_pose(self):
        try:
            x, y, z, roll, pitch, yaw = (
                float(self.custom_pose_entries[label].get())
                for label in ["X (m)", "Y (m)", "Z (m)", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"]
            )
        except ValueError:
            # X/Y/Z are normally already clamped to workspace_bounds by
            # on_entry_commit (slider drag range) by the time this runs -
            # this only fires for genuinely non-numeric text. Written to
            # last_move_to_pose_result (not the label directly) so refresh()
            # doesn't overwrite it with the stale service result a moment
            # later.
            self.handle.last_move_to_pose_result = (False, "Invalid number in custom pose fields")
            return

        qx, qy, qz, qw = _euler_to_quaternion(
            math.radians(roll), math.radians(pitch), math.radians(yaw))
        pose = _pose(x, y, z, qx, qy, qz, qw)
        self.handle.node.request_move_to_custom_pose(self.handle.name, pose)

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
        self.move_to_custom_pose_btn.config(state="normal" if ready else "disabled")
        for btn in self.jog_buttons:
            btn.config(state="normal" if ready else "disabled")
        self.jog_step_entry.config(state="normal" if ready else "disabled")
        self.jog_frame_combo.config(state="readonly" if ready else "disabled")

        self._update_result_label(self.result, "Home", self.handle.last_home_result)
        self._update_result_label(
            self.select_pose_result, "Pose", self.handle.last_select_pose_result)
        self._update_result_label(
            self.move_to_pose_result, "Move To Pose", self.handle.last_move_to_pose_result)
        self._update_result_label(self.jog_result, "Jog", self.handle.last_jog_result)

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