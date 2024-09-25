#!/bin/bash

echo $1 $2 $3

# LEFT
ros2 topic pub --once /covers_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['cover_hinge_joint'], points: [{positions: [1.345], time_from_start: {sec: 2, nanosec: 0}}]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['left_m1st_abase_joint', 'left_a1st_m1st_joint', 'left_a2nd_m2nd_joint'], points: [{positions: [$1, 0, 0], time_from_start: {sec: 1, nanosec: 0}}]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['left_m1st_abase_joint', 'left_a1st_m1st_joint', 'left_a2nd_m2nd_joint'], points: [{positions: [$1, $2, 0], time_from_start: {sec: 1, nanosec: 0}}]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['left_m1st_abase_joint', 'left_a1st_m1st_joint', 'left_a2nd_m2nd_joint'], points: [{positions: [$1, $2, $3], time_from_start: {sec: 1, nanosec: 0}}]}"

ros2 topic pub --once /grippers_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['left_gripper_controller'], points: [{positions: [1.0], time_from_start: {sec: 2} } ]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['left_m1st_abase_joint', 'left_a1st_m1st_joint', 'left_a2nd_m2nd_joint'], points: [{positions: [0, 0, 0], time_from_start: {sec: 3, nanosec: 0}}]}"
ros2 topic pub --once /covers_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['cover_hinge_joint'], points: [{positions: [0], time_from_start: {sec: 3, nanosec: 0}}]}"


# RIGHT
ros2 topic pub --once /covers_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['cover_hinge_joint'], points: [{positions: [1.345], time_from_start: {sec: 2, nanosec: 0}}]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['right_m1st_abase_joint', 'right_a1st_m1st_joint', 'right_a2nd_m2nd_joint'], points: [{positions: [$1, 0, 0], time_from_start: {sec: 1, nanosec: 0}}]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['right_m1st_abase_joint', 'right_a1st_m1st_joint', 'right_a2nd_m2nd_joint'], points: [{positions: [$1, $2, 0], time_from_start: {sec: 1, nanosec: 0}}]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['right_m1st_abase_joint', 'right_a1st_m1st_joint', 'right_a2nd_m2nd_joint'], points: [{positions: [$1, $2, $3], time_from_start: {sec: 1, nanosec: 0}}]}"

ros2 topic pub --once /grippers_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['right_gripper_controller'], points: [{positions: [1.0], time_from_start: {sec: 2} } ]}"
ros2 topic pub --once /manipulators_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['right_m1st_abase_joint', 'right_a1st_m1st_joint', 'right_a2nd_m2nd_joint'], points: [{positions: [0, 0, 0], time_from_start: {sec: 3, nanosec: 0}}]}"
ros2 topic pub --once /covers_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['cover_hinge_joint'], points: [{positions: [0], time_from_start: {sec: 3, nanosec: 0}}]}"

