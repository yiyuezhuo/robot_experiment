# -*- coding: utf-8 -*-
"""
Created on Thu Jun 13 13:34:12 2019

@author: yiyuezhuo
"""
from __future__ import division

import numpy as np

class KalmanFilter:
    def __init__(self, F, B, H, Q, R, X=None, P=None):
        '''
        F: state transition matrix
        B: control matrix
        H: observation matrix
        Q: convariance matrix of process noise
        R: convariance matrix of observation noise
        
        Generative process:
            X[k] = FX[k-1] + BU + E1
            Z[k] = BX[k] + E2
            
            E1 ~ N(0,Q)
            E2 ~ N(0,R)
        '''
        self.F = F
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        
        self.X = X
        self.P = P
        
    def update(self, z):
        y = z - self.H.dot(self.X)
        S = self.H.dot(self.P).dot(self.H.T) + self.R
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
        self.X = self.X + K.dot(y)
        self.P = (np.eye(self.F.shape[0]) - K.dot(self.H)).dot(self.P)
    
    def predict(self,u):
        self.X = self.F.dot(self.X) + self.B.dot(u)
        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
    
    def step(self, u, z):
        self.predict(u)
        self.update(z)
        
    def run(self, u_list, z_list):
        for u,z in zip(u_list, z_list):
            if u is not None:
                self.predict(u)
            if z is not None:
                self.update(z)
            yield self.X, self.P
            
    def __repr__(self):
        return '''
Kalman Filter
  Parameters:
    F=\n{}
    B=\n{}
    H=\n{}
    Q=\n{}
    R=\n{}
  State:
    X=\n{}
    P=\n{}
'''.format(self.F, self.B, self.H, self.Q, self.R, self.X, self.P)
            
class KalmanFilterPosition(KalmanFilter):
    def __init__(self, T, sigma1, sigma2, sigma3, sigma4, X=None,P=None):
        F = np.eye(6)
        F[[0,1,2],[3,4,5]] = T
        
        B = np.zeros([6,3])
        B[[3,4,5],[0,1,2]] = T
        B[[0,1,2,],[0,1,2,]] = T/2
        
        H = np.eye(6)
        
        lt = [0,1,2]
        rb = [3,4,5]
        
        Q = np.eye(6)
        Q[lt, lt] = T*sigma1**2
        Q[rb, rb] = T*sigma2**2
        
        R = np.eye(6)
        R[lt, lt] = T*sigma3**2
        R[rb, rb] = T*sigma4**2
        
        super().__init__(F,B,H,Q,R,X=X,P=P)
        
class DummyFilter:
    def __init__(self):
        self.X = np.zeros(6)
        self.P = np.eye(6)
    def step(self, u, z):
        self.X = z

if __name__ == '__main__':
    # debug for poor
    kalman_filter = KalmanFilterPosition(1/30, 0.1, 0.2, 0.4, 0.5)
    
    a = 1.0
    length = 100
    u_list = np.zeros([length, 3])
    u_list[:,2] = a
    
    t = np.linspace(0,length/30,length)
    
    z_list = np.zeros([length, 6])
    z_list[:,5] = t*a
    z_list[:,2] = 0.5 * a * t**2
    
    kalman_filter.X = np.zeros(6)
    kalman_filter.P = np.eye(6)
    
    X_P_list = list(kalman_filter.run(u_list, z_list))
    
    X_list, P_list = zip(*X_P_list)
    
    import matplotlib.pyplot as plt
    
    
    
    