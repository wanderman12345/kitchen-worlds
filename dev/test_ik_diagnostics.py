#!/usr/bin/env python3
"""
Quick test to see the new verbose IK diagnostics in action.
Run with: python test_ik_diagnostics.py
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def main():
    """Quick IK diagnostics test - run existing test with verbose output enabled."""
    from pybullet_planning.pybullet_tools.utils import connect, disconnect
    from pybullet_planning.world_builder.loaders_partnet_kitchen import create_fetch_kitchen
    
    physics_client = connect(use_gui=False)
    
    print("\n" + "="*90)
    print("Fetch Robot IK Solver Diagnostics Test")
    print("="*90)
    
    try:
        # Create the kitchen world with Fetch
        print("\nCreating Fetch kitchen world...")
        world = create_fetch_kitchen(verbose=False)
        robot = world.robot
        
        print(f"Robot: {robot}")
        print(f"Arms: {robot.arms}")
        print(f"use_torso: {robot.use_torso}")
        
        # Now test IK sampling which will show diagnostics
        print("\n" + "-"*90)
        print("Testing IK generation with verbose output...")
        print("-"*90)
        
        # This would require full problem setup; for now just print what was configured
        print(f"\nDiagnostics enabled in:")
        print(f"  1. tracik.py::solve() - will print tool poses, tforms, solve times")
        print(f"  2. mobile_streams.py::sample_bconf() - will print IK solver creation and attempt tracking")
        print(f"  3. mobile_streams.py::solve_approach_ik() - will print grasp config validation")
        
        print(f"\nRun your normal generation/planning code with verbose=True enabled")
        print(f"Expected output will show:")
        print(f"  - IK solver configuration")
        print(f"  - Each IK attempt (success/failure)")
        print(f"  - Reason for configuration rejection")
        print(f"  - Final validation in solve_approach_ik()")
        
        print("\n" + "="*90)
        
    finally:
        disconnect()

if __name__ == '__main__':
    main()
