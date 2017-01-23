#!/usr/bin/env python
PACKAGE = 'herbpy'
import rospy
import tf
import numpy
import math
from owd_msgs.msg import WAMState

state = {}

def update_tf(msg):
    
    encoders = list(msg.positions)
    
    # Pan
    r_1 = numpy.matrix([[math.cos(encoders[0]),-math.sin(encoders[0]), 0, 0],
                        [math.sin(encoders[0]), math.cos(encoders[0]), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    pan_orientation = tf.transformations.quaternion_from_matrix(r_1)
    
    # Offset between Pan and Tilt
    r_o = numpy.matrix([[1, 0,  0, 0],
                        [0, 0, -1, 0],
                        [0, 1,  0, 0],
                        [0, 0,  0, 1]])
    
    # Tilt
    r_2 = numpy.matrix([[math.cos(encoders[1]),-math.sin(encoders[1]), 0, 0],
                        [math.sin(encoders[1]), math.cos(encoders[1]), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    tilt_orientation = tf.transformations.quaternion_from_matrix(r_o * r_2)
    
    # Broadcast Pan
    state['broadcaster'].sendTransform((0., 0., 0.),
                                       pan_orientation,
                                       rospy.Time.now(),
                                       state['tf_parent'],
                                       state['tf_pan'])
    
    # Broadcast Tilt
    state['broadcaster'].sendTransform((0., 0., 0.),
                                       tilt_orientation,
                                       rospy.Time.now(),
                                       state['tf_pan'],
                                       state['tf_tilt'])

if __name__ == '__main__':
    rospy.init_node('head_tf', anonymous=True)
    wamstate_topic = rospy.get_param('~wamstate_topic', '/head/owd/wamstate')
    state['tf_parent'] = rospy.get_param('~tf_parent', '/head/wam0')
    state['tf_pan'] = rospy.get_param('~tf_pan', 'head/wam1')
    state['tf_tilt'] = rospy.get_param('~tf_tilt', 'head/wam2')
    state['subscriber'] = rospy.Subscriber(wamstate_topic, WAMState, update_tf)
    state['broadcaster'] = tf.TransformBroadcaster()
    rospy.spin()
