#!/usr/bin/env python

# -*- coding: utf-8 -*-
"""
Created on Mon Jun 10 10:00:44 2019

@author: yiyuezhuo
"""

from __future__ import division

from dynamic_drone.dynamic_drone import Drone, PIDControler, DroneController, Observer, DroneControllerNoised
from dynamic_drone.kalman_filter import KalmanFilter, KalmanFilterPosition, DummyFilter


from sensor_msgs.msg import JointState
import rospy
#from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf

import numpy as np


degree = np.pi / 180
broadcaster = tf.TransformBroadcaster()
joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
path_pub = rospy.Publisher('trajectory', Path, queue_size=1)

class DroneRobot:
    def __init__(self, drone_controller, swivel=0.0, swivel_speed = 10.0):
        self.drone_controller = drone_controller
        
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.swivel = swivel
        self.swivel_speed = swivel_speed

        self.path = Path()
        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = 'map'
        
    def step(self, xyz_target, psi_target, dt):
        omega, a_xyz, a_rpy = self.drone_controller.step(xyz_target, psi_target, dt)
        
        joint_state = JointState()
        
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['p1', 'p2', 'p3', 'p4']
        joint_state.position = [self.swivel, self.swivel, self.swivel, self.swivel]
        
        joint_pub.publish(joint_state)
        
        xyz, rpy = self.drone_controller.drone.xyz, self.drone_controller.drone.rpy
        
        broadcaster.sendTransform(
                (xyz[0], xyz[1], xyz[2]),
                tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2]),
                rospy.Time.now(),
                'drone_body',
                'map'
                )
        broadcaster.sendTransform(
                (0.0, 0.0, -0.3),
                tf.transformations.quaternion_from_euler(-np.pi, 0, 0),
                rospy.Time.now(),
                'grenade_body',
                'drone_body'
                )

        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'

        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]

        quat = tf.transformations.quaternion_from_euler(*rpy)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        self.path.poses.append(pose)
        path_pub.publish(self.path)

        self.swivel += degree * self.swivel_speed
        
    def run(self, xyz_target_arr, psi_arr, loop_rate, dt = 1/30,
            keep = True):
        assert xyz_target_arr.shape[0] == psi_arr.shape[0]
        length = xyz_target_arr.shape[0]
        
        for i in range(length):        
            xyz_target = xyz_target_arr[i]
            psi_target = psi_arr[i]
            
            self.step(xyz_target, psi_target, dt)
            
            loop_rate.sleep()
        
        if keep:
            while not rospy.is_shutdown():
                self.step(xyz_target, psi_target, dt)
                
                loop_rate.sleep()
            

    
if __name__ == '__main__':
    rospy.init_node('path_publisher')

    while not rospy.is_shutdown():
        xyz = np.array([10.0, 0.0, 0.0])
        rpy = np.array([0.0, 0.0, 0.0])
        drone = Drone(xyz, rpy, v_xyz=(0,0,0), v_rpy=(0,0,0), 
                         I=(1,1,1), m=1, b=1, l=1, d=1)
        controller_xyz = PIDControler(10.0, 0.001, 1.0)
        controller_rpy = PIDControler(10.0, 0.001, 0.0001)
        #drone_controller = DroneController(drone, controller_xyz, controller_rpy)

        kalman_filter = DummyFilter()

        observer = Observer(kalman_filter, sigma_xyz = 1.0, sigma_v_xyz=1.0, sigma_a_xyz=1.0)

        drone_controller = DroneControllerNoised(drone, controller_xyz, controller_rpy, observer)

        length = 500
        
        xyz_target_arr = np.zeros([length, 3])
        xyz_target_arr[:,2] = 10
        #xyz_target_arr[:,0] = 10
        #xyz_target_arr[:,1] = 10
        #xyz_target_arr[:,0] = 0
        #xyz_target_arr[:,1] = 0
        
        #rpy_target_arr = np.zeros([length, 3])
        psi_target_arr = np.zeros(length)
        
        drone_robot = DroneRobot(drone_controller, swivel_speed = 10.0)
        
        fps = 30
        loop_rate = rospy.Rate(fps)
        drone_robot.run(xyz_target_arr, psi_target_arr, loop_rate, dt=1/fps,keep=False)
        break # run only once

