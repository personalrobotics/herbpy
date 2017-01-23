#! /usr/bin/env python
# coding: utf-8
import rospy
from sensor_msgs.msg import JointState
js = JointState()
js.name = ['/right/j1', '/right/j2', '/right/j3', '/right/j4', '/right/j5', '/right/j6', '/right/j7']
js.position = [3.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

rospy.init_node('joint_sender', anonymous=True)
pub = rospy.Publisher('/rewd_joint_group_position_controller/command', JointState, queue_size=1)
raw_input("Press enter to sent zero-position...")
pub.publish(js)
