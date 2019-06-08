#!/usr/bin/env python

from sensor_msgs.msg import JointState
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf

print 'start python moving script'

if __name__ == '__main__':
    rospy.init_node('state_publisher')

    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    broadcaster = tf.TransformBroadcaster()
    loop_rate = rospy.Rate(30)

    degree = np.pi / 180

    # robot state
    tilt = 0.0
    tinc = degree
    swivel = 0.0
    angle = 0.0
    height = 0.0
    hinc = 0.005

    # message declarations

    joint_state = JointState()

    while not rospy.is_shutdown():
        # update joint_state
        joint_state.header.stamp = rospy.Time.now()
        
        joint_state.name = ['p1', 'p2', 'p3', 'p4']
        joint_state.position = [swivel, swivel, swivel, swivel]

        # update transform
        # (moving in a circle with radius=2)
        # Those logic is transfered into odom_trans.sendTrasform

        # send the joint state and transform
        joint_pub.publish(joint_state)
        broadcaster.sendTransform(
                (np.cos(angle)*3, np.sin(angle)*3, 1.7),
                tf.transformations.quaternion_from_euler(0, 0, angle+np.pi/2),
                rospy.Time.now(),
                'drone_body',
                'map'
                )

        # Create new robot state
        tilt += tinc
        if tilt < -0.5 or tilt > 0:
            tinc *= -1
        height += hinc
        if height > 0.2 or height < 0:
            hinc *= -1
        #swivel += degree
        swivel += degree * 10
        angle += degree/4

        # This will adjust as needed per iteration
        loop_rate.sleep()
