#!/usr/bin/env python3
"""
Pick and place node.

Fixes:
- Avoid calling rclpy.shutdown() from worker thread (prevents Context shutdown error).
- Use correct gripper action name for JointTrajectoryController.
- Avoid MoveIt calls inside subscription callbacks.

Run:
  ~/panda_ws/install/pymoveit2/lib/pymoveit2/pick_and_place.py --ros-args -p use_sim_time:=true -p target_color:=R
"""

from __future__ import annotations

import math
import threading
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from sensor_msgs.msg import JointState
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        self.declare_parameter("target_color", "R")
        self.target_color = str(self.get_parameter("target_color").value).upper().strip()

        self._lock = threading.Lock()
        self._latest_js: JointState | None = None
        self._target_coords: list[float] | None = None
        self._running = False
        self._done = False

        self.callback_group = ReentrantCallbackGroup()

        # MoveIt2 arm interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        self.moveit2.max_velocity = 0.2
        self.moveit2.max_acceleration = 0.2

        # Gripper interface: your gripper_controller is a joint_trajectory_controller
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="/gripper_controller/follow_joint_trajectory",
        )

        # Joint configs
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [
            math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
            math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)
        ]

        # QoS for joint states
        js_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(JointState, "/joint_states", self._js_cb, js_qos)
        self.create_subscription(String, "/color_coordinates", self._coords_cb, 10)

        self.get_logger().info(f"Waiting for target {self.target_color} on /color_coordinates...")
        self.get_logger().info("Waiting for first /joint_states...")

        self.create_timer(0.2, self._maybe_start)
        self.create_timer(0.2, self._check_done)

    def _js_cb(self, msg: JointState):
        with self._lock:
            self._latest_js = msg

    def _coords_cb(self, msg: String):
        with self._lock:
            if self._target_coords is not None:
                return

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()
            if color_id != self.target_color:
                return

            coords = [float(x), float(y), float(z)]
            with self._lock:
                self._target_coords = coords

            self.get_logger().info(
                f"Target {self.target_color} locked at: "
                f"[{coords[0]:.3f}, {coords[1]:.3f}, {coords[2]:.3f}]"
            )
        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates '{msg.data}': {e}")

    def _maybe_start(self):
        with self._lock:
            if self._running:
                return
            if self._latest_js is None or self._target_coords is None:
                return
            self._running = True

        Thread(target=self._run_sequence, daemon=True).start()

    def _check_done(self):
        with self._lock:
            if not self._done:
                return
        # stop spinning cleanly
        self.get_logger().info("Shutting down node cleanly...")
        rclpy.shutdown()

    def _plan_and_execute_joints(self, joints: list[float]) -> bool:
        with self._lock:
            start_js = self._latest_js
        if start_js is None:
            return False

        traj = self.moveit2.plan(joint_positions=joints, start_joint_state=start_js)
        if traj is None:
            self.get_logger().error("Planning failed (joint goal).")
            return False
        self.moveit2.execute(traj)
        return self.moveit2.wait_until_executed()

    def _plan_and_execute_pose(self, position_xyz: list[float], quat_xyzw: list[float]) -> bool:
        with self._lock:
            start_js = self._latest_js
        if start_js is None:
            return False

        traj = self.moveit2.plan(position=position_xyz, quat_xyzw=quat_xyzw, start_joint_state=start_js)
        if traj is None:
            self.get_logger().error("Planning failed (pose goal).")
            return False
        self.moveit2.execute(traj)
        return self.moveit2.wait_until_executed()

    def _run_sequence(self):
        try:
            with self._lock:
                coords = list(self._target_coords)

            pick_position = [coords[0], coords[1], coords[2] - 0.60]
            approach_position = [pick_position[0], pick_position[1], pick_position[2] - 0.31]
            quat_xyzw = [0.0, 1.0, 0.0, 0.0]

            self.get_logger().info("Starting pick-and-place sequence...")

            steps = [
                ("Go to start", lambda: self._plan_and_execute_joints(self.start_joints)),
                ("Go to home",  lambda: self._plan_and_execute_joints(self.home_joints)),
                ("Above target", lambda: self._plan_and_execute_pose(pick_position, quat_xyzw)),
                ("Open gripper", lambda: (self.gripper.open(), self.gripper.wait_until_executed())[1]),
                ("Approach down", lambda: self._plan_and_execute_pose(approach_position, quat_xyzw)),
                ("Close gripper", lambda: (self.gripper.close(), self.gripper.wait_until_executed())[1]),
                ("Back to home", lambda: self._plan_and_execute_joints(self.home_joints)),
                ("Go to drop", lambda: self._plan_and_execute_joints(self.drop_joints)),
                ("Release", lambda: (self.gripper.open(), self.gripper.wait_until_executed())[1]),
                ("Close gripper", lambda: (self.gripper.close(), self.gripper.wait_until_executed())[1]),
                ("Return to start", lambda: self._plan_and_execute_joints(self.start_joints)),
            ]

            for name, fn in steps:
                self.get_logger().info(f"Step: {name}")
                ok = fn()
                if ok is False:
                    self.get_logger().error(f"Aborting: step failed -> {name}")
                    break

            self.get_logger().info("Pick-and-place sequence complete.")

        except Exception as e:
            self.get_logger().error(f"Sequence failed: {e}")
        finally:
            with self._lock:
                self._done = True


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        


if __name__ == "__main__":
    main()