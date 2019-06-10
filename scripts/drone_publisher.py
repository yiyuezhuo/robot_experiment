# -*- coding: utf-8 -*-
"""
Created on Mon Jun 10 10:00:44 2019

@author: yiyuezhuo
"""

from ..dynamic_drone import Drone, PIDControler, DroneController, simulation

'''
xyz = np.array([0.0, 0.0, 0.0])
rpy = np.array([0.0, 0.0, 0.0])
drone = Drone(xyz, rpy, v_xyz=(0,0,0), v_rpy=(0,0,0), 
                 I=(1,1,1), m=1, b=1, l=1, d=1)
controller_xyz = PIDControler(12.0, 0.001, 0.0001)
controller_rpy = PIDControler(12.0, 0.001, 0.0001)
drone_controller = DroneController(drone, controller_xyz, controller_rpy)

length = 500

xyz_target_arr = np.zeros([length, 3])
xyz_target_arr[:,2] = 10
xyz_target_arr[:,0] = 10
xyz_target_arr[:,1] = 10

#rpy_target_arr = np.zeros([length, 3])
psi_target_arr = np.zeros(length)

res = simulation(drone_controller, xyz_target_arr, psi_target_arr)
'''


class DroneRobot:
    pass