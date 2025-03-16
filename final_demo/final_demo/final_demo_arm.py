#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv
import asyncio
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion


def load_poses_from_csv(file_path):
    """
    Load pose data from a CSV file and return a dictionary of Pose objects.
    """
    poses = {}
    with open(file_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            pose = Pose()
            pose.position = Point(
                x=float(row['pos_x']),
                y=float(row['pos_y']),
                z=float(row['pos_z'])
            )
            pose.orientation = Quaternion(
                x=float(row['ori_x']),
                y=float(row['ori_y']),
                z=float(row['ori_z']),
                w=float(row['ori_w'])
            )
            poses[row['name']] = pose
    return poses

def move(entity, pose=None, delay=1):
    """
    Execute a movement or command and then wait for a short delay.
    
    :param entity: An object with an inverse_kinematic_movement() method or a callable (e.g., gripper commands).
    :param pose: A Pose object if applicable.
    :param delay: Time to wait after executing the command.
    """
    entity.inverse_kinematic_movement(pose)
    time.sleep(delay)

def pick_and_place_block(arm, gripper, pick_pose, up_pose, up_pose2, up_pose3, place_pose):
    """
    Perform a pick-and-place operation:
      1. Move above the target (up_pose).
      2. Descend to pick position (pick_pose).
      3. Close the gripper.
      4. Lift back (up_pose).
      5. Move to an intermediate position (up_pose2).
      6. Descend to place position (place_pose).
      7. Open the gripper.
      8. Retract back (up_pose2).
    """
    move(arm, pick_pose)
    gripper.move_to_position(0.65)
    move(arm, up_pose)
    move(arm, up_pose2)
    move(arm, up_pose3)
    move(arm, place_pose)
    gripper.move_to_position(0.0)


def main():
    """
    Main function to initialize the ROS node, load poses from CSV, and perform pick-and-place tasks.
    """
    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    
    # Move arm to a safe vertical/home position.
    arm.go_vertical()
    gripper.move_to_position(0.0)
    # Load poses from the CSV file.
    poses = load_poses_from_csv("/home/zyx119@netid.washington.edu/ros2_ws/src/lab7/lab7/data_final_demo.csv")

    # Execute pick-and-place operations.
    # Example 1: Use green_table_up, green_table_down, red_table_up_1, red_table_down_1.
    pick_and_place_block(arm, gripper, 
                         poses["table_pick"], 
                         poses["table_up"], 
                         poses["turn_to_turtlebot"], 
                         poses["turtlebot_down"],
                         poses["release"])

    
    # Optionally, move the arm back to vertical/home position.
    arm.go_vertical()

    # Shutdown procedures.
    gripper.shutdown()
    arm.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
