from datapoint import DataPoint 
from fusionekf import FusionEKF

def parse_data(file_path):
  """
    Args:
    file_path 
      - path to a text file with all data. 
      - each line should have the following format:
          [SENSOR ID] [SENSOR RAW VALUES] [TIMESTAMP] [GROUND TRUTH VALUES]
          Whereas radar has three measurements (rho, phi, rhodot), lidar has two measurements (x, y).

          Specifically: 
          
          For a row containing radar data, the columns are: 
          sensor_type, rho_measured, phi_measured, rhodot_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth
          
          For a row containing lidar data, the columns are:
          sensor_type, x_measured, y_measured, timestamp, x_groundtruth, y_groundtruth, vx_groundtruth, vy_groundtruth
          
          Example 1: 
          line with three measurements from a radar sensor in polar coordinate 
          followed by a timestamp in unix time 
          followed by the the "ground truth" which is
          actual real position and velocity in cartesian coordinates (four state variables)

          R 8.46642 0.0287602 -3.04035  1477010443399637  8.6 0.25  -3.00029  0
          (R) (rho) (phi) (drho) (timestamp) (real x) (real y) (real vx) (real vy)

          Example 2:
          line with two measurements from a lidar sensor in cartesian coordinates 
          followed by a timestamp in unix time
          followed by the the "ground truth" which is 
          the actual real position and velocity in cartesian coordinates (four state variables)

          L 8.44818 0.251553  1477010443449633  8.45  0.25  -3.00027  0

    Returns:
      all_sensor_data, all_ground_truths
      - two lists of DataPoint() instances 

  """

  all_sensor_data = []
  all_ground_truths = []  

  with open(file_path) as f:
      
    for line in f:
      data = line.split() 
      
      if data[0]  == 'L':
        
        sensor_data = DataPoint({ 
          'timestamp': int(data[3]),
          'name': 'lidar',
          'x': float(data[1]), 
          'y': float(data[2])
        })
        
        g = {'timestamp': int(data[3]),
             'name': 'state',
             'x': float(data[4]),
             'y': float(data[5]),
             'vx': float(data[6]),
             'vy': float(data[7])
        }
          
        ground_truth = DataPoint(g)
                
      elif data[0] == 'R':
          
        sensor_data = DataPoint({ 
          'timestamp': int(data[4]),
          'name': 'radar',
          'rho': float(data[1]), 
          'phi': float(data[2]),
          'drho': float(data[3])
        })
       
        g = {'timestamp': int(data[4]),
             'name': 'state',
             'x': float(data[5]),
             'y': float(data[6]),
             'vx': float(data[7]),
             'vy': float(data[8])
        }
        ground_truth = DataPoint(g)  

      all_sensor_data.append(sensor_data)
      all_ground_truths.append(ground_truth)

  return all_sensor_data, all_ground_truths


def get_state_estimations(EKF, all_sensor_data):
  """
  Calculates all state estimations given a FusionEKF instance() and sensor measurements

  Args:
    EKF - an instance of a FusionEKF() class 
    all_sensor_data - a list of sensor measurements as a DataPoint() instance

  Returns:
    all_state_estimations 
      - a list of all state estimations as predicted by the EKF instance
      - each state estimation is wrapped in  DataPoint() instance
  """

  all_state_estimations = []

  for data in all_sensor_data:
    
    EKF.process(data)
  
    x = EKF.get()
    px, py, vx, vy = x[0, 0], x[1, 0], x[2, 0], x[3, 0]

    g = {'timestamp': data.get_timestamp(),
         'name': 'state',
         'x': px,
         'y': py,
         'vx': vx,
         'vy': vy }

    state_estimation = DataPoint(g)  
    all_state_estimations.append(state_estimation)
    
  return all_state_estimations 

def print_EKF_data(all_sensor_data, all_ground_truths, all_state_estimations, RMSE):
  """
  Prints all relevant EKF data in a nice formal

  Args:
    all_sensor_data
      - a list of sensor measurements as DataPoint() instances
    all_state_estimations
      - a list of state estimations as DataPoint() instances
    all_ground_truths
      - a list of ground truths as DataPoint instances
    RMSE
      - a list of the four computed root-mean-square error of the four state variables considered

    Returns: None 
  """

  px, py, vx, vy = RMSE 

  print("-----------------------------------------------------------")
  print('{:10s} | {:8.3f} | {:8.3f} | {:8.3f} | {:8.3f} |'.format("RMSE:", px, py, vx, vy))
  print("-----------------------------------------------------------")
  print("NUMBER OF DATA POINTS:", len(all_sensor_data))
  print("-----------------------------------------------------------")  

  i = 1
  for s, p, t in zip(all_sensor_data, all_state_estimations, all_ground_truths):
      
    print("-----------------------------------------------------------")
    print("#", i, ":", s.get_timestamp())
    print("-----------------------------------------------------------")  

    if s.get_name() == 'lidar':
      x, y = s.get_raw()
      print('{:15s} | {:8.3f} | {:8.3f} |'.format("LIDAR:", x, y))
    else:
      rho, phi, drho = s.get_raw()
      print('{:15s} | {:8.3f} | {:8.3f} | {:8.3f} |'.format("RADAR:", rho, phi, drho))  

    x, y, vx, vy = p.get()
    print('{:15s} | {:8.3f} | {:8.3f} | {:8.3f} | {:8.3f} |'.format("PREDICTION:", x, y, x, y))  

    x, y, vx, vy = t.get()
    print('{:15s} | {:8.3f} | {:8.3f} | {:8.3f} | {:8.3f} |'.format("TRUTH:", x, y, x, y))  

    i += 1
