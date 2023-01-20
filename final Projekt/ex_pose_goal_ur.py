#!/usr/bin/env python3
"""
Example of moving to a pose goal.
`ros2 run pymoveit2 ex_pose_goal_ur.py --ros-args -p position:="[0.25, 0.0, 1.0]" -p quat_xyzw:="[0.0, 0.0, 0.0, 1.0]" -p cartesian:=False`
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5e


def main():

    rclpy.init()

    # Create node for this example
    node = Node("ex_pose_goal_ur")

    # Declare parameters for position and orientation
    node.declare_parameter("position_pose1", [0.5, 0.0, 0.4])
    node.declare_parameter("quat_xyzw_pose1", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian_pose1", False)

    node.declare_parameter("position_pose2", [0.5, 0.0, 0.2])
    node.declare_parameter("quat_xyzw_pose2", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian_pose2", False)

    node.declare_parameter("position_pose3", [0.5, 0.3, 0.4])
    node.declare_parameter("quat_xyzw_pose3", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian_pose3", False)

    node.declare_parameter("position_pose4", [0.5, 0.3, 0.2])
    node.declare_parameter("quat_xyzw_pose4", [1.0, 0.0, 0.0, 0.0])
    node.declare_parameter("cartesian_pose4", False)


    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5e.joint_names(),
        base_link_name=ur5e.base_link_name(),
        end_effector_name=ur5e.end_effector_name(),
        group_name=ur5e.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    position_pose1 = node.get_parameter("position_pose1").get_parameter_value().double_array_value
    quat_xyzw_pose1 = node.get_parameter("quat_xyzw_pose1").get_parameter_value().double_array_value
    cartesian_pose1 = node.get_parameter("cartesian_pose1").get_parameter_value().bool_value

    position_pose2 = node.get_parameter("position_pose2").get_parameter_value().double_array_value
    quat_xyzw_pose2 = node.get_parameter("quat_xyzw_pose2").get_parameter_value().double_array_value
    cartesian_pose2 = node.get_parameter("cartesian_pose2").get_parameter_value().bool_value

    position_pose3 = node.get_parameter("position_pose3").get_parameter_value().double_array_value
    quat_xyzw_pose3 = node.get_parameter("quat_xyzw_pose3").get_parameter_value().double_array_value
    cartesian_pose3 = node.get_parameter("cartesian_pose3").get_parameter_value().bool_value

    position_pose4 = node.get_parameter("position_pose4").get_parameter_value().double_array_value
    quat_xyzw_pose4 = node.get_parameter("quat_xyzw_pose4").get_parameter_value().double_array_value
    cartesian_pose4 = node.get_parameter("cartesian_pose4").get_parameter_value().bool_value

    # Move to pose
    node.get_logger().info(
        f"Moving to {{position: {list(position_pose1)}, quat_xyzw: {list(quat_xyzw_pose1)}}}"
    )
    moveit2.move_to_pose(position=position_pose1, quat_xyzw=quat_xyzw_pose1, cartesian=cartesian_pose1)
    moveit2.wait_until_executed()

    node.get_logger().info(
        f"Moving to {{position: {list(position_pose2)}, quat_xyzw: {list(quat_xyzw_pose2)}}}"
    )
    moveit2.move_to_pose(position=position_pose2, quat_xyzw=quat_xyzw_pose2, cartesian=cartesian_pose2)
    moveit2.wait_until_executed()

    node.get_logger().info(
        f"Moving to {{position: {list(position_pose1)}, quat_xyzw: {list(quat_xyzw_pose1)}}}"
    )
    moveit2.move_to_pose(position=position_pose1, quat_xyzw=quat_xyzw_pose1, cartesian=cartesian_pose1)
    moveit2.wait_until_executed()

    node.get_logger().info(
        f"Moving to {{position: {list(position_pose3)}, quat_xyzw: {list(quat_xyzw_pose3)}}}"
    )
    moveit2.move_to_pose(position=position_pose3, quat_xyzw=quat_xyzw_pose3, cartesian=cartesian_pose3)
    moveit2.wait_until_executed()

    node.get_logger().info(
        f"Moving to {{position: {list(position_pose4)}, quat_xyzw: {list(quat_xyzw_pose4)}}}"
    )
    moveit2.move_to_pose(position=position_pose4, quat_xyzw=quat_xyzw_pose4, cartesian=cartesian_pose4)
    moveit2.wait_until_executed()

    node.get_logger().info(
        f"Moving to {{position: {list(position_pose3)}, quat_xyzw: {list(quat_xyzw_pose3)}}}"
    )
    moveit2.move_to_pose(position=position_pose3, quat_xyzw=quat_xyzw_pose3, cartesian=cartesian_pose3)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
