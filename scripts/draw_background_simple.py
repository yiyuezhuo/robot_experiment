#!/usr/bin/env python

from visualization_msgs.msg import Marker
import rospy
import numpy as np
#import time

print 'start updating background marker'

if __name__ == '__main__':
    
    rospy.init_node('background_drawer')

    loop_rate = rospy.Rate(30)
    
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time() # t=0 instead of rospy.Time.now()
    
     
    #marker.ns = "basic_shapes"
    marker.id = 3
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    
    marker.lifetime = rospy.Duration()
    
    background_pub = rospy.Publisher("visualization_marker", Marker, queue_size=100)
    
    while not rospy.is_shutdown():
        background_pub.publish(marker)
        loop_rate.sleep()
    
    #time.sleep(5)
