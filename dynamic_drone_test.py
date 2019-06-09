# -*- coding: utf-8 -*-
"""
Created on Sun Jun  9 10:42:18 2019

@author: yiyuezhuo
"""

from dynamic_drone import Drone
import numpy as np

import matplotlib.pyplot as plt

xyz = np.array([0.0, 0.0, 0.0])
rpy = np.array([0.0, 0.0, 0.0])
drone = Drone(xyz, rpy, v_xyz=(0,0,0), v_rpy=(0,0,0), 
                 I=(1,1,1), m=1, b=1, l=1, d=1)

def simulation(omega_arr, dt = 1/30, show=True, callback=None):
    '''
    omega_arr: length x 4 matrix
    '''
    
    length = omega_arr.shape[0]
    xyz_arr = np.zeros([length, 3])
    rpy_arr = np.zeros([length, 3])
    v_xyz_arr = np.zeros([length, 3])
    v_rpy_arr = np.zeros([length, 3])
    a_xyz_arr = np.zeros([length, 3])
    a_rpy_arr = np.zeros([length, 3])
    
    #dt=1/30
    
    for i in range(length):
        omega = np.array([1.0, 1.0, 1.0, 1.0]) * 2
        a_xyz, a_rpy = drone.step(omega, dt)
        
        xyz_arr[i,:] = drone.xyz
        rpy_arr[i,:] = drone.rpy
        v_xyz_arr[i,:] = drone.v_xyz
        v_rpy_arr[i,:] = drone.v_rpy
        a_xyz_arr[i,:] = a_xyz
        a_rpy_arr[i,:] = a_rpy
        
        if callback:
            callback(drone.xyz, drone.rpy, drone.v_xyz, drone.v_rpy, a_xyz, a_rpy)
    
    
    name_list = ['X','Y','Z',
                 'r','p','y',
                 "v_X","v_Y","v_Z",
                 "v_r","v_p","v_y",
                 'a_X','a_Y','a_Z',
                 'a_r','a_p','a_y']
    arr_list = [xyz_arr[:,0], xyz_arr[:,1], xyz_arr[:,2],
                rpy_arr[:,0], rpy_arr[:,1], rpy_arr[:,2],
                v_xyz_arr[:,0], v_xyz_arr[:,1], v_xyz_arr[:,2],
                v_rpy_arr[:,0], v_rpy_arr[:,1], v_rpy_arr[:,2],
                a_xyz_arr[:,0], a_xyz_arr[:,1], a_xyz_arr[:,2],
                a_rpy_arr[:,0], a_rpy_arr[:,1], a_rpy_arr[:,2]]
    
    for name, arr in zip(name_list, arr_list):
        plt.plot(arr)
        plt.title(name)
        plt.show()
            
    return {'name_list': name_list,
            'arr_list': arr_list,
            'xyz_arr': xyz_arr,
            'rpy_arr': rpy_arr,
            'v_xyz_arr': v_xyz_arr,
            'v_rpy_arr': v_rpy_arr,
            'a_xyz_arr': a_xyz_arr,
            'a_rpy_arr': a_rpy_arr}


length = 1000
omega_arr = np.ones([length, 4]) * 2

res = simulation(omega_arr)