import numpy as np 
from datapoint import DataPoint
from tools import cartesian_to_polar
from helpers import parse_data

def get_all_differences(all_sensor_data, all_ground_truths):

  pxs, pys, vxs, vys, rhos, phis, drhos = [], [], [], [], [], [], []

  for s, t in zip(all_sensor_data, all_ground_truths):
        
    if s.get_name() == 'lidar':
    
      spx, spy, _, _ = s.get()
      tpx, tpy, _, _ = t.get()

      pxs += [spx - tpx]
      pys += [spy - tpy]
      
    else:
        
      spx, spy, svx, svy = s.get()
      tpx, tpy, tvx, tvy = t.get()

      srho, sphi, sdrho = s.get_raw()
      trho, tphi, tdrho = cartesian_to_polar(tpx, tpy, tvx, tvy)
        
      pxs += [spx - tpx]
      pys += [spy - tpy]        
      vxs += [svx - tvx]
      vys += [svy - tvy]
      rhos += [srho - trho]
      phis += [sphi - tphi]
      drhos += [sdrho - tdrho]

    
  return pxs, pys, vxs, vys, rhos, phis, drhos

def get_variance(x):
  return np.var(np.array(x))

def print_variances(pxs, pys, vxs, vys, rhos, phis, drhos):
  print("x:", get_variance(pxs))
  print("y:", get_variance(pys))
  print("vx:", get_variance(vxs))
  print("vy:", get_variance(vys))
  print("rho:", get_variance(rhos))
  print("phi:", get_variance(phis))
  print("drho:", get_variance(drhos))
  print()

if __name__ == "__main__":

  all_sensor_data1, all_ground_truths1 = parse_data("data/data-1.txt")
  pxs1, pys1, vxs1, vys1, rhos1, phis1, drhos1 = get_all_differences(all_sensor_data1, all_ground_truths1)

  print("Variances from: data-1.txt")
  print_variances(pxs1, pys1, vxs1, vys1, rhos1, phis1, drhos1)
  
  all_sensor_data2, all_ground_truths2 = parse_data("data/data-2.txt")
  pxs2, pys2, vxs2, vys2, rhos2, phis2, drhos2 = get_all_differences(all_sensor_data2, all_ground_truths2)

  print("Variances from: data-2.txt")
  print_variances(pxs2, pys2, vxs2, vys2, rhos2, phis2, drhos2)

  print("Combined variances")  
  print_variances(pxs1 + pxs2, pys1 + pys2, vxs1 + vxs2, 
                  vys1 + vys2, rhos1 + rhos2, phis1 + phis2, drhos1 + drhos2)