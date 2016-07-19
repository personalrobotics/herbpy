#!/usr/bin/env python
# coding: utf-8

import ros_control_client_py
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

rospy.init_node("trajectory_test")


client = ros_control_client_py.FollowJointTrajectoryClient("/right_trajectory_controller/follow_joint_trajectory")

raw_input("Press enter to execute trajectory...")

joint_state = rospy.wait_for_message("/joint_states", JointState, timeout=1.0)

names = [
  '/right/j1', 
  '/right/j2', 
  '/right/j3', 
  '/right/j4', 
  '/right/j5', 
  '/right/j6', 
  '/right/j7'
]

cur_state = filter(lambda t: t[0] in names, zip(joint_state.name, joint_state.position))

assert len(cur_state) is len(names)

cur_names, cur_pos = zip(*cur_state) # unzip
end_pos = []

for p in cur_pos:
  end_pos.append(p + 0.2)

traj_msg = JointTrajectory()
traj_msg.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(2.0)
traj_msg.joint_names = cur_names
traj_msg.points = []

orig_point = JointTrajectoryPoint()
orig_point.positions = cur_pos
orig_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
orig_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
orig_point.time_from_start = rospy.Duration.from_sec(0.0)

traj_msg.points.append(orig_point)
end_point = JointTrajectoryPoint()
end_point.positions = end_pos
end_point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_point.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
end_point.time_from_start = rospy.Duration.from_sec(2.0)

traj_msg.points.append(end_point)
print traj_msg

traj_f = client.execute(traj_msg)
print traj_f.result()

