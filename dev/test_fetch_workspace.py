#!/usr/bin/env python3
"""
Quick diagnostic to check if Fetch tool poses are geometrically feasible.
Tests if requested grasps are even reachable for Fetch kinematics.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

def test_fetch_workspace():
    """Check Fetch arm workspace and reachability."""
    from pybullet_tools.utils import connect, disconnect, load_robot, Point
    from pybullet_planning.robot_builder.robots import FetchRobot
    import math
    
    physics_client = connect(use_gui=False)
    
    try:
        robot = load_robot('fetch')
        robot.use_torso = True
        
        print("\n" + "="*80)
        print("Fetch Robot Workspace Analysis")
        print("="*80)
        
        # Get arm info
        print("\nRobot Info:")
        print(f"  Body ID: {robot.body}")
        print(f"  Arms: {robot.arms}")
        print(f"  Base joints: {robot.base_joints}")
        print(f"  use_torso: {robot.use_torso}")
        
        # Get tool link
        arm = robot.arms[0]
        tool_link = robot.get_tool_link(arm)
        print(f"\n  Arm '{arm}' tool link: {tool_link}")
        
        # Check arm joints and limits
        arm_joints = robot.get_arm_joints(arm)
        print(f"\n  Arm joints ({len(arm_joints)}):")
        from pybullet_tools.utils import get_joint_name, get_joint_limits
        for j_idx, j in enumerate(arm_joints):
            j_name = get_joint_name(robot.body, j)
            lower, upper = get_joint_limits(robot.body, j)
            print(f"    {j_idx}: {j_name} [{lower:.3f}, {upper:.3f}]")
        
        # Check custom limits
        print(f"\n  Custom limits: {robot.custom_limits}")
        
        # Check base joints
        print(f"\n  Base joints ({len(robot.base_joints)}):")
        for j_idx, j in enumerate(robot.base_joints):
            j_name = get_joint_name(robot.body, j)
            lower, upper = get_joint_limits(robot.body, j)
            print(f"    {j_idx}: {j_name} [{lower:.3f}, {upper:.3f}]")
        
        # Estimate workspace
        print("\nWorkspace Estimation:")
        print(f"  Shoulder to wrist length (rough): ~0.8m")
        print(f"  Base to shoulder offset: ~0.2m (approx)")
        print(f"  Rough max reach from base: ~1.0-1.2m")
        print(f"  Torso height range: ~0.2m (z-axis)")
        
        # Check if IKSolver can be created
        print("\nIK Solver Creation:")
        try:
            from pybullet_tools.tracik import IKSolver
            ik_solver = IKSolver(robot.body, tool_link=tool_link,
                                custom_limits=robot.custom_limits)
            print(f"  IKSolver created successfully")
            print(f"  Base link: {ik_solver.base_name}")
            print(f"  Tool link: {ik_solver.tool_name}")
            print(f"  Joints in chain ({len(ik_solver.joints)}):")
            joint_names = ik_solver.joint_names
            for j_name in joint_names:
                print(f"    {j_name}")
            print(f"  Joint limits: {ik_solver.joint_limits}")
        except Exception as e:
            print(f"  ERROR creating IKSolver: {e}")
            import traceback
            traceback.print_exc()
        
        print("\n" + "="*80)
        
    finally:
        disconnect()

if __name__ == '__main__':
    test_fetch_workspace()
