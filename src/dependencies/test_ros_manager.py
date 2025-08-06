#!/usr/bin/env python3
"""
Test script for ROSLaunchManager class.
Demonstrates usage patterns and error handling.
"""

import time
from dependencies.ros_manager import ROSLaunchManager, ROSStatus


def main():
    """Test the ROSLaunchManager functionality"""
    
    print("=== ROS Launch Manager Test ===\n")
    
    # Create manager instance
    ros_manager = ROSLaunchManager("wx250")
    
    # 1. Check if ROS2 is already running
    print("1. Checking if ROS2 is already running...")
    if ros_manager.check_ros2_running():
        print("   ✓ ROS2 processes detected")
    else:
        print("   ✗ No ROS2 processes detected")
    
    # 2. Check initial status
    print("\n2. Initial status check...")
    status, error = ros_manager.get_status()
    print(f"   Status: {status.value}")
    if error:
        print(f"   Error: {error}")
    
    # 3. Start ROS launch
    print("\n3. Starting ROS launch process...")
    if ros_manager.start_ros_launch():
        print("   ✓ ROS launch started successfully")
    else:
        print("   ✗ Failed to start ROS launch")
        status, error = ros_manager.get_status()
        if error:
            print(f"   Error: {error}")
        return
    
    # 4. Monitor status for a few seconds
    print("\n4. Monitoring status...")
    for i in range(5):
        time.sleep(1)
        status, error = ros_manager.get_status()
        print(f"   [{i+1}/5] Status: {status.value}")
        if error:
            print(f"   Error: {error}")
        
        if status == ROSStatus.ERROR:
            print("   Process encountered an error!")
            break
    
    # 5. Check if process is running
    print(f"\n5. Is process running? {ros_manager.is_running()}")
    
    # 6. Get process output (if any)
    print("\n6. Checking process output...")
    stdout, stderr = ros_manager.get_process_output()
    if stdout:
        print(f"   Recent stdout: {stdout[-200:]}")  # Last 200 chars
    if stderr:
        print(f"   Recent stderr: {stderr[-200:]}")  # Last 200 chars
    
    # 7. Stop the process
    print("\n7. Stopping ROS launch process...")
    if ros_manager.stop_ros_launch():
        print("   ✓ ROS launch stopped successfully")
    else:
        print("   ✗ Failed to stop ROS launch")
        status, error = ros_manager.get_status()
        if error:
            print(f"   Error: {error}")
    
    # 8. Final status check
    print("\n8. Final status check...")
    status, error = ros_manager.get_status()
    print(f"   Status: {status.value}")
    if error:
        print(f"   Error: {error}")
    
    print("\n=== Test Complete ===")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest failed with exception: {e}")
