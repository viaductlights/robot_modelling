#!/usr/bin/env python3
"""
Multi-Robot Control GUI
tkinter + rclpy (ROS2 Jazzy)

  - rclpy spins in a background thread, tkinter owns the main thread
  - "Home" and "Run Task" go through the /<name>/go_home and /<name>/run_task
    Trigger services (task_coordinator/hmi_motion_server -> MoveIt-planned,
    collision-checked)
  - Both buttons are only enabled once their service is available (backend ready)
  - EVERY rclpy call happens on the executor thread (marshalled via a queue)
  - home_pose comes from the SRDF (single source of truth)
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


# ---- Configuration --------------------------------------------------------
ROBOT_CONFIG = [
    {"name": "nemo", "joint_states_topic": "/nemo/joint_states"},
    {"name": "bean", "joint_states_topic": "/bean/joint_states"},
]
STALE_AFTER_SEC = 1.0   # no new JointState message for longer than this -> "offline"
ROS_TICK_SEC = 0.1      # ROS timer: refresh readiness + drain the queue
# --------------------------------------------------------------------------


class RobotHandle:
    """Encapsulates everything for ONE robot. rclpy objects are only
    touched by the ROS thread; the GUI only reads cached attributes."""

    def __init__(self, node: Node, name: str, joint_states_topic: str):
        self.node = node
        self.name = name

        self.latest_state = None
        self.last_stamp = 0.0

        self._planner_ready = False              # ROS thread writes, GUI thread reads
        self.last_home_result = None             # (success|None, message) | None
        self.last_task_result = None             # (success|None, message) | None

        self.sub = node.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, 10)
        self.home_client = node.create_client(Trigger, f"/{name}/go_home")
        self.task_client = node.create_client(Trigger, f"/{name}/run_task")

    # ---- ROS thread --------------------------------------------------------
    def _on_joint_state(self, msg: JointState):
        self.latest_state = msg
        self.last_stamp = self.node.get_clock().now().nanoseconds * 1e-9

    def refresh_readiness(self):
        self._planner_ready = (
            self.home_client.service_is_ready()
            and self.task_client.service_is_ready())

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
            cfg["name"]: RobotHandle(self, cfg["name"], cfg["joint_states_topic"])
            for cfg in ROBOT_CONFIG
        }
        self._cmd_queue = queue.Queue()
        self.create_timer(ROS_TICK_SEC, self._ros_tick)
        self.get_logger().info(f"GUI node started, robots: {list(self.robots)}")

    # ---- called from the GUI (queue only, thread-safe) --------------------
    def request_go_home(self, name: str):
        self._cmd_queue.put(("home", name))

    def request_run_task(self, name: str):
        self._cmd_queue.put(("task", name))

    # ---- ROS thread --------------------------------------------------------
    def _ros_tick(self):
        for handle in self.robots.values():
            handle.refresh_readiness()
        self._drain_commands()

    def _drain_commands(self):
        while True:
            try:
                action, name = self._cmd_queue.get_nowait()
            except queue.Empty:
                break
            handle = self.robots.get(name)
            if handle is None:
                continue
            client = handle.home_client if action == "home" else handle.task_client
            if not handle.planner_ready():
                self._set_result(handle, action, (False, "Planner not ready"))
                continue
            self._set_result(handle, action, (None, "running..."))
            future = client.call_async(Trigger.Request())
            future.add_done_callback(partial(self._on_command_done, handle, action))

    def _set_result(self, handle: RobotHandle, action: str, result):
        if action == "home":
            handle.last_home_result = result
        else:
            handle.last_task_result = result

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
        self.task_btn = ttk.Button(
            btns, text="Run Task",
            command=lambda: handle.node.request_run_task(handle.name),
            state="disabled")
        self.task_btn.pack(side="left", padx=2)

        self.result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.result.pack(anchor="w")
        self.task_result = tk.Label(self, text="", fg="#555", font=("TkDefaultFont", 9))
        self.task_result.pack(anchor="w")

    def refresh(self):
        online = self.handle.is_online()
        self.status.config(
            text="\u25cf online" if online else "\u25cf offline",
            fg="#27ae60" if online else "#c0392b")

        ready = self.handle.planner_ready()
        self.planner_status.config(
            text="Planner: ready" if ready else "Planner: waiting for go_home service...",
            fg="#27ae60" if ready else "#c0392b")
        self.home_btn.config(state="normal" if ready else "disabled")
        self.task_btn.config(state="normal" if ready else "disabled")

        res = self.handle.last_home_result
        if res is None:
            self.result.config(text="")
        else:
            ok, msg = res
            color = "#555" if ok is None else ("#27ae60" if ok else "#c0392b")
            self.result.config(text=f"Home: {msg}", fg=color)

        task_res = self.handle.last_task_result
        if task_res is None:
            self.task_result.config(text="")
        else:
            ok, msg = task_res
            color = "#555" if ok is None else ("#27ae60" if ok else "#c0392b")
            self.task_result.config(text=f"Task: {msg}", fg=color)

        self.joints.config(state="normal")
        self.joints.delete("1.0", "end")
        st = self.handle.latest_state
        if st is None:
            self.joints.insert("end", "waiting for JointState...")
        else:
            for n, p in zip(st.name, st.position):
                self.joints.insert("end", f"{n:<24}{p:+.4f}\n")
        self.joints.config(state="disabled")


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