#!/usr/bin/env python

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import rospy
import numpy as np
import tf
#import time

print 'start updating background marker'

def create_mesh_marker(id,x=0,y=0,z=0,a=1,r=0,g=0,b=0, mesh="package://mydrone/meshs/type59.dae"):
    marker = Marker()
    marker.header.frame_id = '/map'
    marker.header.stamp = rospy.Time() # t=0 instead of rospy.Time.now()


    #marker.ns = "basic_shapes"
    marker.id = id
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD

    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z

    rpy = np.array([0.,0.,np.pi/2])
    quat = tf.transformations.quaternion_from_euler(*rpy)

    marker.pose.orientation.x = quat[0]
    marker.pose.orientation.y = quat[1]
    marker.pose.orientation.z = quat[2]
    marker.pose.orientation.w = quat[3]

    marker.mesh_resource = mesh#"package://mydrone/meshs/type59.dae"
    
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    marker.color.a = a
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    
    marker.lifetime = rospy.Duration()

    return marker

def create_tank_marker(*args, **kwargs):
    return create_mesh_marker(*args, r=1.0, mesh="package://mydrone/meshs/type59.dae", **kwargs)

def create_bandit_marker(*args, **kwargs):
    return create_mesh_marker(*args, a=0.5, b=1.0, mesh="package://mydrone/meshs/bandit.dae", **kwargs)


if __name__ == '__main__':
    
    rospy.init_node('background_drawer')

    loop_rate = rospy.Rate(3)
    
    #marker = create_tank_marker(0,0)
    tank_markers = [create_tank_marker(i, x=-i*10) for i in range(1)]
    bandit_marker = create_bandit_marker(4, x=10)

    background_pub = rospy.Publisher("visualization_marker", Marker, queue_size=100)
    background_list_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size = 100)
    
    while not rospy.is_shutdown():
        #background_pub.publish(marker)
        marker_array = MarkerArray()
        #marker_array.markers = [marker]
        marker_array.markers = tank_markers + [bandit_marker]
        background_list_pub.publish(marker_array)
        loop_rate.sleep()
    
    #time.sleep(5)
