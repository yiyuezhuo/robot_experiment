# -*- coding: utf-8 -*-
"""
Created on Sun Jun  9 09:46:12 2019

@author: yiyuezhuo

Reference:
    https://wenku.baidu.com/view/4b5a7e7ca4e9856a561252d380eb6294dd882291.html
    https://blog.csdn.net/linxiaobo110/article/details/89890970
    https://github.com/linxiaobo110/QuadrotorFly

    Fuck GFW, in 6.4 I can only search for those chinese materials and even
    can't read content such as wikipedia even.
"""

from __future__ import division

import numpy as np
from numpy import sin,cos,arcsin

class Drone:
    def __init__(self, xyz, rpy, v_xyz=(0,0,0), v_rpy=(0,0,0), 
                 I=(1,1,1), m=1, b=1, l=1, d=1):
        '''
        Initial state parameters:
            xyz: (x,y,z) initial 3d position vector
            rpy: (r,p,y) initial 3d angular vector (\phi, \theta, \psi)
            v_xyz: (x',y',z') initial 3d position speed vector
            v_rpu: (r',y',z') initial 3d angular speed vector
            
        Drone parameters:
            I: (Ix,Iy,Iz) 3d inertia vector representing a diagonal matrix
            m: mass
            b: Lift coefficient of rotor
            l: length of arm
            d: drag coefficient of rotor
        '''
        self.xyz = np.array(xyz,dtype=float)
        self.rpy = np.array(rpy,dtype=float)
        self.v_xyz = np.array(v_xyz,dtype=float)
        self.v_rpy = np.array(v_rpy,dtype=float)
        
        self.I = I
        self.m = m
        self.b = b
        self.l = l
        self.d = d
        
    def get_acceleration(self, omega):
        '''
        Return ideal acceleration
        omega: argular speed of four rotors.
        '''
        F = omega**2 * self.b
        
        U1 = F.sum()
        U2 = self.l * (F[3] - F[1])
        U3 = self.l * (F[2] - F[0])
        U4 = self.d * (-omega[0]**2 + omega[1]**2 - omega[2]**2 + omega[3]**2)
        
        phi, theta, psi = self.rpy
        m = self.m
        g = 9.8
        
        a_x = (sin(theta)*cos(phi)*cos(psi) + sin(phi)*sin(psi)) * U1/m
        a_y = (sin(theta)*cos(phi)*sin(psi) - sin(phi)*cos(psi)) * U1/m
        a_z = cos(phi)*cos(theta)*U1/m - g
        
        v_phi, v_theta, v_psi = self.v_rpy
        Ix,Iy,Iz = self.I
        
        a_phi = (Iy-Iz)/Ix * v_theta * v_psi + U2/Ix
        a_theta = (Iz - Ix)/Iy * v_phi * v_psi + U3/Iy
        a_psi = (Ix - Iy)/Iz * v_phi * v_theta + U4/Iz
        
        a_xyz = np.array([a_x, a_y, a_z])
        a_rpy = np.array([a_phi, a_theta, a_psi])
        
        return a_xyz, a_rpy
    
    def step(self, omega, dt, noise=None):
        '''
        omega: argular speed of four rotors.
        '''
        a_xyz, a_rpy = self.get_acceleration(omega)
        
        if noise:
            a_xyz, a_rpy = noise(a_xyz, a_rpy)
        
        self.xyz = self.xyz + (self.v_xyz + a_xyz/2) * dt
        self.rpy = self.rpy + (self.v_rpy + a_rpy/2) * dt
        #self.xyz = self.xyz + self.v_xyz * dt + a_xyz/2 * dt**2
        #self.rpy = self.rpy + self.v_rpy * dt + a_rpy/2 * dt**2
        
        self.v_xyz = self.v_xyz + a_xyz * dt
        self.v_rpy = self.v_rpy + a_rpy * dt
        
        return a_xyz, a_rpy
        
class PIDControler:
    def __init__(self, kp, ki, kd, track_length = 100, log=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.error_list = []
        self.track_length = track_length
        
        self.log = log
        if self.log:
            self.log_list = []
        
    def step(self, x_obs, x_target, dt):
        e = x_target - x_obs
        
        tp = self.kp * e
        if len(self.error_list) >0:
            ti = self.ki * np.array(self.error_list).sum(axis=0) *dt
        else:
            ti = np.zeros_like(tp)
        if len(self.error_list)>0:
            td = self.kd * (self.kd * e - self.error_list[-1]) / dt
        else:
            td = np.zeros_like(tp)
        
        self.error_list.append(e)
        if len(self.error_list) > self.track_length:
            self.error_list = self.error_list[-self.track_length:]
        
        #print(tp,ti,td)
        
        if self.log:
            self.log_list.append((tp,ti,td))
            
        return tp + ti + td

class DroneController:
    def __init__(self, drone, controller_xyz, controller_rpy, log=True):
        self.drone = drone
        self.controller_xyz = controller_xyz
        self.controller_rpy = controller_rpy
        
        b,l,d = self.drone.b, self.drone.l, self.drone.d
        mat = np.array([[b,b,b,b],
                        [0,-l*b,0,l*b],
                        [-l*b,0,l*b,0],
                        [-d,d,-d,d]])
        self.U_to_omega2 = np.linalg.inv(mat)
        
        self.log = log
        if self.log:
            self.log_list = []
    
    def step(self, xyz_target, psi_target, dt):
        xyz, rpy = self.drone.xyz, self.drone.rpy
        #dprint('true',xyz, rpy)
        phi, theta, psi = rpy
        I,m,l = self.drone.I, self.drone.m, self.drone.l
        
        a_xyz_target = self.controller_xyz.step(xyz, xyz_target, dt)
        
        U1 = (a_xyz_target[2] + 9.8)*m/(cos(phi)*cos(theta))
                
        
        sin_phi0 = m*(a_xyz_target[0]*sin(psi) - a_xyz_target[1]*cos(psi))/U1
        phi0 = arcsin(np.clip(sin_phi0, -np.pi/6, np.pi/6))
        sin_theta0 = (a_xyz_target[0]*m*cos(psi) + a_xyz_target[1]*sin(psi))/(U1*cos(phi0))
        theta0 = arcsin(np.clip(sin_theta0, -np.pi/6, np.pi/6))
        
        rpy_target = np.array([phi0, theta0, psi_target])
        
        a_rpy_target = self.controller_rpy.step(rpy, rpy_target, dt)
        
        U = np.array([
                U1,
                a_rpy_target[0] * I[0]/l,
                a_rpy_target[1] * I[1]/l,
                a_rpy_target[2] * I[2]/l
                ])
        #omega2 = self.U_to_omega2 @ U
        # Provide shit python2 support required by ROS
        omega2 = self.U_to_omega2.dot(U)
        
        omega = np.sqrt(np.clip(omega2, 0, np.inf))
        #print(U, omega2, omega)
        if self.log:
            self.log_list.append((U, omega2, omega))
        
        a_xyz, a_rpy = self.drone.step(omega, dt)
        
        return omega, a_xyz, a_rpy

class DroneControllerNoised:
    def __init__(self, drone, controller_xyz, controller_rpy, observer, log=True):
        self.drone = drone
        self.controller_xyz = controller_xyz
        self.controller_rpy = controller_rpy
        
        b,l,d = self.drone.b, self.drone.l, self.drone.d
        mat = np.array([[b,b,b,b],
                        [0,-l*b,0,l*b],
                        [-l*b,0,l*b,0],
                        [-d,d,-d,d]])
        self.U_to_omega2 = np.linalg.inv(mat)
        
        self.log = log
        if self.log:
            self.log_list = []
            
        self.observer = observer
        
    def step(self, xyz_target, psi_target, dt):
        #xyz, rpy = self.drone.xyz, self.drone.rpy
        xyz,rpy = self.observer.observe()
        #print(xyz,rpy)
        
        phi, theta, psi = rpy
        I,m,l = self.drone.I, self.drone.m, self.drone.l
        
        a_xyz_target = self.controller_xyz.step(xyz, xyz_target, dt)
        
        U1 = (a_xyz_target[2] + 9.8)*m/(cos(phi)*cos(theta))
                
        
        sin_phi0 = m*(a_xyz_target[0]*sin(psi) - a_xyz_target[1]*cos(psi))/U1
        phi0 = arcsin(np.clip(sin_phi0, -np.pi/6, np.pi/6))
        sin_theta0 = (a_xyz_target[0]*m*cos(psi) + a_xyz_target[1]*sin(psi))/(U1*cos(phi0))
        theta0 = arcsin(np.clip(sin_theta0, -np.pi/6, np.pi/6))
        
        rpy_target = np.array([phi0, theta0, psi_target])
        
        a_rpy_target = self.controller_rpy.step(rpy, rpy_target, dt)
        
        U = np.array([
                U1,
                a_rpy_target[0] * I[0]/l,
                a_rpy_target[1] * I[1]/l,
                a_rpy_target[2] * I[2]/l
                ])
        #omega2 = self.U_to_omega2 @ U
        # Provide shit python2 support required by ROS
        omega2 = self.U_to_omega2.dot(U)
        
        omega = np.sqrt(np.clip(omega2, 0, np.inf))
        #print(U, omega2, omega)
        if self.log:
            self.log_list.append((U, omega2, omega))
        
        a_xyz, a_rpy = self.drone.step(omega, dt)
        
        self.observer.update(self.drone.xyz, self.drone.rpy,
                             self.drone.v_xyz, self.drone.v_rpy,
                             a_xyz, a_rpy)
        #print(self.observer.observe())
        
        return omega, a_xyz, a_rpy

class Observer:
    '''
    Observer class wrap noise generator and filter
    '''
    def __init__(self, filter, sigma_xyz = 0.0, sigma_rpy = 0.0, 
                 sigma_v_xyz=0.0, sigma_v_rpy = 0.0,
                 sigma_a_xyz=0.0, sigma_a_rpy = 0.0,
                 rpy_ideal = None):
        
        self.filter = filter
        
        if rpy_ideal is None:
            self.rpy_ideal = np.zeros(3)
        else:
            self.rpy_ideal = rpy_ideal
        
        self.sigma_xyz = sigma_xyz
        self.sigma_rpy = sigma_rpy
        self.sigma_v_xyz = sigma_v_xyz
        self.sigma_v_rpy = sigma_v_rpy
        self.sigma_a_xyz = sigma_a_xyz
        self.sigma_a_rpy = sigma_a_rpy
        
    def observe(self):
        '''
        return filtered value of xyz, rpy
        '''
        X = self.filter.X
        return X[:3],self.rpy_ideal
    
    def apply_noise(self, xyz, rpy, v_xyz, v_rpy, a_xyz, a_rpy):
        xyz = xyz + np.random.randn(*xyz.shape) * self.sigma_xyz
        rpy = rpy + np.random.randn(*rpy.shape) * self.sigma_rpy
        v_xyz = v_xyz + np.random.randn(*v_xyz.shape) * self.sigma_v_xyz
        v_rpy = v_rpy + np.random.randn(*v_rpy.shape) * self.sigma_v_rpy
        a_xyz = a_xyz + np.random.randn(*a_xyz.shape) * self.sigma_a_xyz
        a_rpy = a_rpy + np.random.randn(*a_rpy.shape) * self.sigma_a_rpy
        return xyz, rpy, v_xyz, v_rpy, a_xyz, a_rpy
    
    def update(self, xyz, rpy, v_xyz, v_rpy, a_xyz, a_rpy):
        xyz, rpy, v_xyz, v_rpy, a_xyz, a_rpy = self.apply_noise(
                xyz, rpy, v_xyz, v_rpy, a_xyz, a_rpy)
        
        u = a_xyz
        z = np.concatenate([xyz, v_xyz], axis=0)
        self.filter.step(u,z)
        
        self.rpy_ideal = rpy
        