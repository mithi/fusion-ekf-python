from kalmanfilter import KalmanFilter
from datapoint import DataPoint 
from tools import calculate_jacobian, cartesian_to_polar, time_difference
import numpy as np 

class FusionEKF:
  """
  A class that gets sensor measurements from class DataPoint 
  and predicts the next state of the system using an extended Kalman filter algorithm

  The state variables we are considering in this system are the position and velocity
  in x and y cartesian coordinates, in essence there are 4 variables we are tracking.
  
  In particular, an instance of this class gets measurements from both lidar and radar sensors
  lidar sensors measure positions in cartesian coordinates (2 values)
  radar sensors measure position and velocity in polar coordinates (3 values)

  lidar sensor are linear and radar sensors are non-linear, so we use the jacobian algorithm
  to compute the state transition matrix H unlike a simple kalman filter.
  """

  def __init__(self, d):
    self.initialized = False
    self.timestamp = 0
    self.n = d['number_of_states']
    self.P = d['initial_process_matrix']
    self.F = d['inital_state_transition_matrix']
    self.Q = d['initial_noise_matrix']
    self.radar_R = d['radar_covariance_matrix']
    self.lidar_R = d['lidar_covariance_matrix']
    self.lidar_H = d['lidar_transition_matrix']
    self.a = (d['acceleration_noise_x'], d['acceleration_noise_y'])
    self.kalmanFilter = KalmanFilter(self.n)

  def updateQ(self, dt):
    
    dt2 = dt * dt
    dt3 = dt * dt2
    dt4 = dt * dt3
    
    x, y = self.a
    
    r11 = dt4 * x / 4
    r13 = dt3 * x / 2
    r22 = dt4 * y / 4
    r24 = dt3 * y /  2
    r31 = dt3 * x / 2 
    r33 = dt2 * x
    r42 = dt3 * y / 2
    r44 = dt2 * y
    
    Q = np.matrix([[r11, 0, r13, 0],
                  [0, r22, 0, r24],
                  [r31, 0, r33, 0], 
                  [0, r42, 0, r44]])
    
    self.kalmanFilter.setQ(Q)
    
  def update(self, data):
    
    dt = time_difference(self.timestamp, data.get_timestamp())
    self.timestamp = data.get_timestamp()
        
    self.kalmanFilter.updateF(dt)
    self.updateQ(dt)
    self.kalmanFilter.predict()
    
    z = np.matrix(data.get_raw()).T
    x = self.kalmanFilter.getx()
    
    if data.get_name() == 'radar':        
      
      px, py, vx, vy = x[0, 0], x[1, 0], x[2, 0], x[3, 0]
      rho, phi, drho = cartesian_to_polar(px, py, vx, vy)
      H = calculate_jacobian(px, py, vx, vy)
      Hx = (np.matrix([[rho, phi, drho]])).T
      R = self.radar_R 
         
    elif data.get_name() == 'lidar':
    
      H = self.lidar_H
      Hx = self.lidar_H * x
      R = self.lidar_R

    self.kalmanFilter.update(z, H, Hx, R)

  def start(self, data):
    
    self.timestamp = data.get_timestamp()
    x = np.matrix([data.get()]).T
    self.kalmanFilter.start(x, self.P, self.F, self.Q)
    self.initialized = True
    
  def process(self, data):
    
    if self.initialized: 
      self.update(data)
    else:
      self.start(data)

  def get(self):
    return self.kalmanFilter.getx()