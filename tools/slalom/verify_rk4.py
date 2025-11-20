import sys
import os
import numpy as np
import math

# Add the directory to sys.path
sys.path.append("/home/naoto/Desktop/mouse/Astraea/tools/slalom")

from slalom import Slalom

def verify():
    v = 1000 # 1000 mm/s
    rad = 50
    n = 2
    ang = 90
    end_pos = {"x": 45, "y": 45}
    slip_gain = 250
    type = "normal"
    K = 50
    list_K_y = [50.0]

    print("Running Euler...")
    sla_euler = Slalom(v, rad, n, ang, end_pos, slip_gain, type, K, list_K_y, method="euler")
    sla_euler.calc_base_time()
    res_euler = sla_euler.calc(0)
    
    print("Running RK4...")
    sla_rk4 = Slalom(v, rad, n, ang, end_pos, slip_gain, type, K, list_K_y, method="rk4")
    sla_rk4.calc_base_time()
    res_rk4 = sla_rk4.calc(0)

    print(f"Euler Final X: {res_euler['x'][-1]:.6f}, Y: {res_euler['y'][-1]:.6f}")
    print(f"RK4 Final X:   {res_rk4['x'][-1]:.6f}, Y: {res_rk4['y'][-1]:.6f}")
    
    diff_x = abs(res_euler['x'][-1] - res_rk4['x'][-1])
    diff_y = abs(res_euler['y'][-1] - res_rk4['y'][-1])
    
    print(f"Diff X: {diff_x:.6f}")
    print(f"Diff Y: {diff_y:.6f}")

    if diff_x < 1e-9 and diff_y < 1e-9:
        print("WARNING: Results are identical. RK4 might not be working or dt is too small/Euler is too good.")
    else:
        print("Results differ as expected.")

    # Verify Slip
    print("\nRunning Euler Slip...")
    res_slip_euler = sla_euler.calc_slip(0)
    print("Running RK4 Slip...")
    res_slip_rk4 = sla_rk4.calc_slip(0)
    
    print(f"Euler Slip Final X: {res_slip_euler['x'][-1]:.6f}, Y: {res_slip_euler['y'][-1]:.6f}")
    print(f"RK4 Slip Final X:   {res_slip_rk4['x'][-1]:.6f}, Y: {res_slip_rk4['y'][-1]:.6f}")

    diff_slip_x = abs(res_slip_euler['x'][-1] - res_slip_rk4['x'][-1])
    diff_slip_y = abs(res_slip_euler['y'][-1] - res_slip_rk4['y'][-1])
    
    print(f"Diff Slip X: {diff_slip_x:.6f}")
    print(f"Diff Slip Y: {diff_slip_y:.6f}")

    if diff_slip_x < 1e-9 and diff_slip_y < 1e-9:
        print("WARNING: Slip results are identical.")
    else:
        print("Slip results differ as expected.")

if __name__ == "__main__":
    verify()
