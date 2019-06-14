# -*- coding: utf-8 -*-
"""
Created on Mon Jun 10 10:42:21 2019

@author: yiyuezhuo
"""

import numpy as np
import matplotlib.pyplot as plt


def simulation(drone_controller, xyz_target_arr, psi_arr, 
               dt = 1/30, show=True, callback=None,
               length = None):
    '''
    omega_arr: length x 4 matrix
    '''
    if not hasattr(xyz_target_arr, '__call__') and not hasattr(xyz_target_arr, '__call__'):
        assert xyz_target_arr.shape[0] == psi_arr.shape[0]
        length = xyz_target_arr.shape[0]
    else:
        assert length is not None
    
    #length = omega_arr.shape[0]
    xyz_arr = np.zeros([length, 3])
    rpy_arr = np.zeros([length, 3])
    v_xyz_arr = np.zeros([length, 3])
    v_rpy_arr = np.zeros([length, 3])
    a_xyz_arr = np.zeros([length, 3])
    a_rpy_arr = np.zeros([length, 3])
    omega_arr = np.zeros([length, 4])
    
    #dt=1/30
    drone = drone_controller.drone
    
    for i in range(length):
        #omega = np.array([1.0, 1.0, 1.0, 1.0]) * 2
        #omega = omega_arr[i]
        #a_xyz, a_rpy = drone.step(omega, dt)
        
        if hasattr(xyz_target_arr, '__call__'):
            xyz_target = xyz_target_arr(drone.xyz, drone.rpy, drone.v_xyz, drone.rpy)
        else:
            xyz_target = xyz_target_arr[i]
        if hasattr(psi_arr, '__call__'):
            psi_target = psi_arr(drone.xyz, drone.rpy, drone.v_xyz, drone.rpy)
        else:
            psi_target = psi_arr[i]
        
        omega, a_xyz, a_rpy = drone_controller.step(xyz_target, psi_target, dt)
        
        
        
        xyz_arr[i,:] = drone.xyz
        rpy_arr[i,:] = drone.rpy
        v_xyz_arr[i,:] = drone.v_xyz
        v_rpy_arr[i,:] = drone.v_rpy
        a_xyz_arr[i,:] = a_xyz
        a_rpy_arr[i,:] = a_rpy
        omega_arr[i,:] = omega
        
        if callback:
            callback(drone.xyz, drone.rpy, drone.v_xyz, drone.v_rpy, a_xyz, a_rpy)
    
    
    name_list = ['X','Y','Z',
                 'r','p','y',
                 "v_X","v_Y","v_Z",
                 "v_r","v_p","v_y",
                 'a_X','a_Y','a_Z',
                 'a_r','a_p','a_y',
                 'omega1','omega2','omega3','omega4']
    arr_list = [xyz_arr[:,0], xyz_arr[:,1], xyz_arr[:,2],
                rpy_arr[:,0], rpy_arr[:,1], rpy_arr[:,2],
                v_xyz_arr[:,0], v_xyz_arr[:,1], v_xyz_arr[:,2],
                v_rpy_arr[:,0], v_rpy_arr[:,1], v_rpy_arr[:,2],
                a_xyz_arr[:,0], a_xyz_arr[:,1], a_xyz_arr[:,2],
                a_rpy_arr[:,0], a_rpy_arr[:,1], a_rpy_arr[:,2],
                omega_arr[:,0], omega_arr[:,1], omega_arr[:,2], omega_arr[:,3]]
    
    
    for name, arr in zip(name_list, arr_list):
        if isinstance(show, (list, tuple, set)) or not show:
            if name not in show:
                continue
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
            'a_rpy_arr': a_rpy_arr,
            'omega_arr': omega_arr}

def plot_pretty(res):
    fig, axs = plt.subplots(nrows=7, ncols=4, figsize=(12,24))
    i=0
    for prefix in ['','v_','a_']:
        for target in ['xyz','rpy']:
            key = '{}{}_arr'.format(prefix, target)
            arr = res[key]
            for j in range(3):
                axs[i,j].plot(arr[:,j])
                axs[i,j].set_title('{}{}-{}'.format(prefix, target, target[j]))
            i+=1
    
    for j in range(4):
        axs[6,j].plot(res['omega_arr'][:,j])
        axs[6,j].set_title('omega {}'.format(j+1))
        
        