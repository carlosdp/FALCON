#!/usr/bin/env python3

import time
import yaml
import numpy as np
from utils.comm.state_processor.unitree import UnitreeStateProcessor

def main():
    # Load configuration
    config_path = "config/g1/g1_29dof.yaml"
    with open(config_path, "r") as file:
        config = yaml.safe_load(file)
    
    # Create Unitree state processor
    state_processor = UnitreeStateProcessor(config)
    
    print(f"Initialized UnitreeStateProcessor for robot type: {config['ROBOT_TYPE']}")
    print(f"Number of DOF: {state_processor.num_dof}")
    print(f"Number of motors: {state_processor.num_motor}")
    print("\nStarting state monitoring (printing every 0.5 seconds)...")
    print("=" * 80)
    
    try:
        while True:
            # Get current state data
            state_data = state_processor.get_robot_state_data()
            
            if state_data is not None:
                # Parse state components
                q = state_processor.q  # positions (base + joints)
                dq = state_processor.dq  # velocities 
                tau_est = state_processor.tau_est  # estimated torques
                ddq = state_processor.ddq  # accelerations
                
                print(f"\nTimestamp: {time.time():.3f}")
                print("-" * 40)
                
                # Base state
                print("Base Position (x,y,z):", q[0:3])
                print("Base Quaternion (w,x,y,z):", q[3:7])
                print("Base Linear Velocity:", dq[0:3])
                print("Base Angular Velocity:", dq[3:6])
                print("Base Linear Acceleration:", ddq[0:3])
                
                # Joint states
                print("\nJoint Positions:", q[7:])
                print("Joint Velocities:", dq[6:])
                print("Joint Torques (estimated):", tau_est[6:])
                
                # Summary statistics
                print("\nSummary:")
                print(f"  Max joint position: {np.max(np.abs(q[7:])):.3f}")
                print(f"  Max joint velocity: {np.max(np.abs(dq[6:])):.3f}")
                print(f"  Max joint torque: {np.max(np.abs(tau_est[6:])):.3f}")
                print(f"  IMU acceleration magnitude: {np.linalg.norm(ddq[0:3]):.3f}")
                
                print("=" * 80)
            else:
                print("No state data available yet...")
            
            # Sleep for 0.5 seconds
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n\nStopping state monitoring...")
        print("Exiting cleanly.")

if __name__ == "__main__":
    main()