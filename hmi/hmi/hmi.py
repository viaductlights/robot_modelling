#!/usr/bin/env python3
"""
Multi-Robot Control GUI - Grundgeruest
tkinter + rclpy (ROS2 Jazzy)

Architektur:
  - rclpy spinnt im Hintergrund-Thread, tkinter im Haupt-Thread
  - pro Roboter ein RobotHandle (Subscriptions, letzter Zustand,
    spaeter: Command-Clients fuer move_group / Controller)
  - GUI pollt den gecachten Zustand via root.after() (thread-safe Lesen)
"""

import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# ---- Konfiguration: hier Roboter / Topics anpassen -----------------------
# ZUM TESTEN VOR DEM PREFIXING (Single-Robot-Demo):
#   nemo1 -> "joint_states_topic": "/joint_states"
#   bean2 bleibt dann einfach "offline" - das ist erwartet.
ROBOT_CONFIG = [
    {
        "name": "nemo",
        "joint_states_topic": "/nemo/joint_states",
        "trajectory_topic": "/nemo/robot_controller/joint_trajectory",
        "home": {
            "J1": 0.0,
            "J2": 0.0,
            "J3": 0.0,
            "J4": 0.0,
            "J5": 0.0,
            "J6": 0.0,
            "J7": 0.0,
        },
    },
    {
        "name": "bean",
        "joint_states_topic": "/bean/joint_states",
        "trajectory_topic": "/bean/robot_controller/joint_trajectory",
        "home": {
            "joint_one": 0.0,
            "joint_two": 0.0,
            "joint_three": 0.0,
            "joint_four": 0.0,
            "joint_five": 0.0,
            "joint_six": 0.0,
        },
    },
]
STALE_AFTER_SEC = 1.0   # keine neue Nachricht laenger als das -> "offline"
# --------------------------------------------------------------------------


class RobotHandle:
    """Kapselt alles zu EINEM Roboter.
    Erweiterungspunkt: hier kommen spaeter die Command-Clients rein
    (pymoveit2 action client, JointTrajectory publisher, ...)."""

    def __init__(self, node: Node, name: str, joint_states_topic: str,
                 trajectory_topic: str, home: dict):
        self.node = node
        self.name = name
        self.home = home
        self.latest_state = None
        self.last_stamp = 0.0

        self.sub = node.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, 10)

        self.traj_pub = node.create_publisher(
            JointTrajectory, trajectory_topic, 10)

    def _on_joint_state(self, msg: JointState):
        # laeuft im ROS-Thread - nur Attribute setzen, KEINE Widgets anfassen
        self.latest_state = msg
        self.last_stamp = self.node.get_clock().now().nanoseconds * 1e-9

    def is_online(self) -> bool:
        if self.latest_state is None:
            return False
        now = self.node.get_clock().now().nanoseconds * 1e-9
        return (now - self.last_stamp) < STALE_AFTER_SEC

    # ---- Platzhalter fuer echte Funktionen (spaeter erweitern) -----------
    def go_home(self):
        msg = JointTrajectory()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.joint_names = list(self.home.keys())

        point = JointTrajectoryPoint()
        point.positions = list(self.home.values())
        point.velocities = [0.0] * len(point.positions)
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)
        self.traj_pub.publish(msg)

        self.node.get_logger().info(
            f"[{self.name}] Home gesendet: {msg.joint_names} -> {point.positions}"
    )

    def print_state(self):
        if self.latest_state is None:
            self.node.get_logger().info(f"[{self.name}] noch kein JointState empfangen")
            return
        pairs = ", ".join(
            f"{n}={p:.3f}"
            for n, p in zip(self.latest_state.name, self.latest_state.position))
        self.node.get_logger().info(f"[{self.name}] {pairs}")


class MultiRobotNode(Node):
    """Eine Node, die alle Roboter-Handles haelt. Single-threaded Executor reicht."""

    def __init__(self):
        super().__init__("hmi")
        self.robots = {
            cfg["name"]: RobotHandle(
                self,
                cfg["name"],
                cfg["joint_states_topic"],
                cfg["trajectory_topic"],
                cfg["home"],
            )
    for cfg in ROBOT_CONFIG
}
        self.get_logger().info(f"GUI-Node gestartet, Roboter: {list(self.robots)}")


class RobotPanel(ttk.LabelFrame):
    """GUI-Panel fuer einen Roboter. Erweiterungspunkt: mehr Buttons hier rein."""

    def __init__(self, parent, handle: RobotHandle):
        super().__init__(parent, text=handle.name, padding=10)
        self.handle = handle

        # tk.Label statt ttk.Label, damit die Statusfarbe sicher greift
        self.status = tk.Label(self, text="\u25cf offline", fg="#c0392b",
                               font=("TkDefaultFont", 10, "bold"))
        self.status.pack(anchor="w")

        self.joints = tk.Text(self, height=8, width=40, state="disabled",
                              font=("TkFixedFont", 9))
        self.joints.pack(pady=6)

        btns = ttk.Frame(self)
        btns.pack(anchor="w")
        ttk.Button(btns, text="Print State",
                   command=handle.print_state).pack(side="left", padx=2)
        ttk.Button(btns, text="Home (TODO)",
                   command=handle.go_home).pack(side="left", padx=2)

    def refresh(self):
        online = self.handle.is_online()
        self.status.config(
            text="\u25cf online" if online else "\u25cf offline",
            fg="#27ae60" if online else "#c0392b")

        self.joints.config(state="normal")
        self.joints.delete("1.0", "end")
        st = self.handle.latest_state
        if st is None:
            self.joints.insert("end", "warte auf JointState...")
        else:
            for n, p in zip(st.name, st.position):
                self.joints.insert("end", f"{n:<24}{p:+.4f}\n")
        self.joints.config(state="disabled")


class App:
    def __init__(self, node: MultiRobotNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("nemo1 / bean2 - Control (Grundgeruest)")

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
        # laeuft im Haupt-Thread - hier DARF auf Widgets zugegriffen werden
        for panel in self.panels:
            panel.refresh()
        self.root.after(100, self._refresh_loop)   # 10 Hz GUI-Update

    def _on_close(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = MultiRobotNode()

    # ROS2 dreht im Hintergrund, tkinter besitzt den Haupt-Thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = App(node)
    app.run()           # blockiert bis Fenster geschlossen wird

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()