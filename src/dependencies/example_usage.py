#!/usr/bin/env python3
"""
Example usage of ROSLaunchManager for common scenarios.
"""

import time
from ros_status import ROSLaunchManager, ROSStatus


def example_basic_usage():
    """Basic usage example"""
    print("=== Basic Usage Example ===")
    
    # Create manager
    manager = ROSLaunchManager("wx250")
    
    # Start process
    if manager.start_ros_launch():
        print("ROS launched successfully!")
        
        # Let it run for a bit
        time.sleep(5)
        
        # Stop process
        manager.stop_ros_launch()
        print("ROS stopped")
    else:
        status, error = manager.get_status()
        print(f"Failed to start: {error}")


def example_with_monitoring():
    """Example with status monitoring"""
    print("\n=== Monitoring Example ===")
    
    manager = ROSLaunchManager("wx250")
    
    # Check if ROS is already running elsewhere
    if manager.check_ros2_running():
        print("Warning: ROS2 processes already detected")
    
    # Start and monitor
    if manager.start_ros_launch():
        print("Monitoring ROS process...")
        
        for i in range(10):
            time.sleep(1)
            
            if not manager.is_running():
                status, error = manager.get_status()
                print(f"Process stopped unexpectedly: {error}")
                break
            
            print(f"  [{i+1}/10] Process running...")
        
        # Clean shutdown
        manager.stop_ros_launch()


def example_error_handling():
    """Example with comprehensive error handling"""
    print("\n=== Error Handling Example ===")
    
    manager = ROSLaunchManager("invalid_robot_model")  # This should fail
    
    try:
        if not manager.start_ros_launch():
            status, error = manager.get_status()
            print(f"Expected failure: {error}")
        
        # Even if it fails, try to clean up
        manager.stop_ros_launch()
        
    except Exception as e:
        print(f"Caught exception: {e}")


def example_restart():
    """Example showing restart functionality"""
    print("\n=== Restart Example ===")
    
    manager = ROSLaunchManager("wx250")
    
    # Start
    if manager.start_ros_launch():
        print("Initial start successful")
        time.sleep(2)
        
        # Restart
        print("Restarting...")
        if manager.restart():
            print("Restart successful")
            time.sleep(2)
            manager.stop_ros_launch()
        else:
            status, error = manager.get_status()
            print(f"Restart failed: {error}")


if __name__ == "__main__":
    example_basic_usage()
    example_with_monitoring()
    example_error_handling()
    example_restart()
