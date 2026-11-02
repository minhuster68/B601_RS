#!/usr/bin/env python3
"""Plan and execute named arm/gripper goals against the running mock demo."""

import argparse
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState


def wait(node, future, timeout=45.0):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError("Timed out waiting for MoveIt")
    return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan-only", action="store_true", help="Validate planning without execution")
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("rebot_moveit_smoke_test")
    client = ActionClient(node, MoveGroup, "/move_action")
    positions = {}
    subscription = node.create_subscription(
        JointState, "/joint_states", lambda msg: positions.update(zip(msg.name, msg.position)), 10,
    )
    try:
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("MoveIt is unavailable; start demo.launch.py first")
        deadline = time.monotonic() + 10
        while len(positions) < 8 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(positions) < 8:
            raise RuntimeError("Did not receive all eight joint states from mock hardware")

        share = Path(get_package_share_directory("rebot_arm_moveit"))
        srdf = ET.parse(share / "config" / "rebot_arm_rs.srdf").getroot()
        for group, target in (("arm", "ready"), ("arm", "home"), ("gripper", "open"), ("gripper", "closed")):
            state = next(s for s in srdf.findall("group_state")
                         if s.get("name") == target and s.get("group") == group)
            goal = MoveGroup.Goal()
            goal.request.group_name = group
            goal.request.pipeline_id = "ompl"
            goal.request.planner_id = "RRTConnectkConfigDefault"
            goal.request.num_planning_attempts = 5
            goal.request.allowed_planning_time = 5.0
            goal.request.max_velocity_scaling_factor = 0.5
            goal.request.max_acceleration_scaling_factor = 0.5
            goal.request.start_state.is_diff = True
            constraints = Constraints()
            for joint in state.findall("joint"):
                constraints.joint_constraints.append(JointConstraint(
                    joint_name=joint.get("name"), position=float(joint.get("value")),
                    tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0,
                ))
            goal.request.goal_constraints = [constraints]
            goal.planning_options.plan_only = args.plan_only
            goal.planning_options.planning_scene_diff.is_diff = True
            goal.planning_options.planning_scene_diff.robot_state.is_diff = True
            handle = wait(node, client.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError(f"MoveIt rejected {group}/{target}")
            result_future = handle.get_result_async()
            try:
                result = wait(node, result_future).result
            except RuntimeError:
                wait(node, handle.cancel_goal_async(), timeout=5.0)
                raise
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f"{group}/{target} failed: MoveIt code {result.error_code.val}")
            if not args.plan_only:
                expected = {j.get("name"): float(j.get("value")) for j in state.findall("joint")}
                if group == "gripper":
                    expected["gripper_joint2"] = expected["gripper_joint1"]
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    if all(abs(positions.get(name, float("inf")) - value) < 0.002 for name, value in expected.items()):
                        break
                else:
                    raise RuntimeError(f"Joint feedback did not reach {group}/{target}: {positions}")
            print(f"PASS: {group}/{target} {'planned' if args.plan_only else 'planned, executed and confirmed by joint feedback'}", flush=True)
    finally:
        node.destroy_subscription(subscription)
        client.destroy()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
