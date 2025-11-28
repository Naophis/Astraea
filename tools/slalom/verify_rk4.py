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
    tgt_ang = 90 # Renamed from 'ang' to 'tgt_ang' for clarity
    start_ang = 0 # Added initial angle
    end_pos = {"x": 45, "y": 45}
    slip_gain = 250
    type = "normal"
    K = 50
    list_K_y = [50.0]

    print("Running Verification for all combinations...")

    combinations = [
        ("euler", "euler"),
        ("rk4", "euler"),
        ("euler", "rk4"),
        ("rk4", "rk4")
    ]

    results = {}

    for method, method_w in combinations:
        print(f"\nTesting Position: {method}, Angular Velocity: {method_w}")
        
        # Non-Slip
        sla = Slalom(v, rad, n, tgt_ang, end_pos, slip_gain, type, K, list_K_y, method=method, method_w=method_w)
        sla.calc_base_time()
        res = sla.calc(start_ang)
        final_x = res["x"][-1]
        final_y = res["y"][-1]
        print(f"  Non-Slip Final X: {final_x:.6f}, Y: {final_y:.6f}")
        results[(method, method_w, "nonslip")] = (final_x, final_y)

        # Slip
        sla_slip = Slalom(v, rad, n, tgt_ang, end_pos, slip_gain, type, K, list_K_y, method=method, method_w=method_w)
        sla_slip.calc_base_time()
        res_slip = sla_slip.calc_slip(start_ang)
        final_slip_x = res_slip["x"][-1]
        final_slip_y = res_slip["y"][-1]
        print(f"  Slip Final X: {final_slip_x:.6f}, Y: {final_slip_y:.6f}")
        results[(method, method_w, "slip")] = (final_slip_x, final_slip_y)

    # Compare RK4/RK4 vs Euler/Euler
    e_e_ns = results[("euler", "euler", "nonslip")]
    r_r_ns = results[("rk4", "rk4", "nonslip")]
    print(f"\nDiff Non-Slip (Euler/Euler vs RK4/RK4): X: {abs(e_e_ns[0]-r_r_ns[0]):.6f}, Y: {abs(e_e_ns[1]-r_r_ns[1]):.6f}")

    # Compare RK4/Euler vs Euler/Euler (Should be close but different due to pos integration)
    r_e_ns = results[("rk4", "euler", "nonslip")]
    print(f"Diff Non-Slip (Euler/Euler vs RK4/Euler): X: {abs(e_e_ns[0]-r_e_ns[0]):.6f}, Y: {abs(e_e_ns[1]-r_e_ns[1]):.6f}")

    # Compare Euler/RK4 vs Euler/Euler (Should be close but different due to w integration)
    e_r_ns = results[("euler", "rk4", "nonslip")]
    print(f"Diff Non-Slip (Euler/Euler vs Euler/RK4): X: {abs(e_e_ns[0]-e_r_ns[0]):.6f}, Y: {abs(e_e_ns[1]-e_r_ns[1]):.6f}")

    # Compare RK4/RK4 vs Euler/Euler for Slip
    e_e_s = results[("euler", "euler", "slip")]
    r_r_s = results[("rk4", "rk4", "slip")]
    print(f"\nDiff Slip (Euler/Euler vs RK4/RK4): X: {abs(e_e_s[0]-r_r_s[0]):.6f}, Y: {abs(e_e_s[1]-r_r_s[1]):.6f}")

    # Compare RK4/Euler vs Euler/Euler for Slip
    r_e_s = results[("rk4", "euler", "slip")]
    print(f"Diff Slip (Euler/Euler vs RK4/Euler): X: {abs(e_e_s[0]-r_e_s[0]):.6f}, Y: {abs(e_e_s[1]-r_e_s[1]):.6f}")

    # Compare Euler/RK4 vs Euler/Euler for Slip
    e_r_s = results[("euler", "rk4", "slip")]
    print(f"Diff Slip (Euler/Euler vs Euler/RK4): X: {abs(e_e_s[0]-e_r_s[0]):.6f}, Y: {abs(e_e_s[1]-e_r_s[1]):.6f}")

    print("\nTesting Turn Time Integration (RK4 vs Euler)...")
    # Test Turn Time RK4 (Position/W Euler)
    sla_time_rk4 = Slalom(v, rad, n, tgt_ang, end_pos, slip_gain, type, K, list_K_y, method="euler", method_w="euler", method_time="rk4")
    sla_time_rk4.calc_base_time()
    res_time_rk4 = sla_time_rk4.calc(start_ang)
    final_x_time = res_time_rk4["x"][-1]
    final_y_time = res_time_rk4["y"][-1]
    print(f"  Et (RK4): {sla_time_rk4.Et}")
    print(f"  Final X (Time RK4): {final_x_time:.6f}, Y: {final_y_time:.6f}")
    
    # Compare with Euler/Euler/Euler
    sla_time_euler = Slalom(v, rad, n, tgt_ang, end_pos, slip_gain, type, K, list_K_y, method="euler", method_w="euler", method_time="euler")
    print(f"  Et (Euler): {sla_time_euler.Et}")
    print(f"  Diff Et: {abs(sla_time_rk4.Et - sla_time_euler.Et):.9f}")
    print(f"  Diff X: {abs(final_x_time - e_e_ns[0]):.6f}, Y: {abs(final_y_time - e_e_ns[1]):.6f}")

if __name__ == "__main__":
    verify()
