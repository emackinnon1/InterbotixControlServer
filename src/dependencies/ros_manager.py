# create dependency to double check ROS status before running commands
import subprocess
import time
import signal
import os
import logging
from enum import Enum
from typing import Optional, Tuple


class ROSStatus(Enum):
    """Enum for ROS process status"""
    NOT_RUNNING = "not_running"
    RUNNING = "running"
    STARTING = "starting"
    ERROR = "error"
    STOPPING = "stopping"


class ROSLaunchManager:
    """
    Manages ROS launch processes with status checking and error handling.
    """
    
    def __init__(self, robot_model: str = "wx250"):
        self.logger = logging.getLogger(__name__)
        self.robot_model = robot_model
        self.process: Optional[subprocess.Popen] = None
        self.status = ROSStatus.NOT_RUNNING
        self.error_message: Optional[str] = None
        self.launch_command = [
            "ros2",
            "launch", 
            "interbotix_xsarm_control", 
            "xsarm_control.launch.py", 
            f"robot_model:={robot_model}"
        ]
        self.ros_root = "/opt/ros/humble"
    
    def check_ros2_running(self) -> bool:
        """
        Check if ROS2 processes are currently running using ps aux.
        
        Returns:
            bool: True if ROS2 processes are found, False otherwise
        """
        try:
            result = subprocess.run(
                ["ps", "aux"], 
                capture_output=True, 
                text=True, 
                timeout=10
            )
            
            # Check for ROS-related processes
            ros_processes = subprocess.run(
                ["grep", "-i", "ros"],
                input=result.stdout,
                capture_output=True,
                text=True
            )
            
            # Filter out this script's own process
            filtered_output = []
            for line in ros_processes.stdout.splitlines():
                if "ros_manager.py" not in line and "grep -i ros" not in line:
                    filtered_output.append(line)
            
            return len(filtered_output) > 0
            
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError) as e:
            error_msg = f"Failed to check ROS status: {str(e)}"
            self.logger.error(error_msg)
            self.error_message = error_msg
            return False
    
    def start_ros_launch(self) -> bool:
        """
        Start the ROS launch process in the background.
        
        Returns:
            bool: True if process started successfully, False otherwise
        """
        if self.is_running():
            error_msg = "ROS launch process is already running"
            self.logger.error(error_msg)
            self.error_message = error_msg
            return False
        
        try:
            self.status = ROSStatus.STARTING
            self.error_message = None
            self.logger.info("Starting ROS launch process...")
            
            # Start the process in the background
            self.process = subprocess.Popen(
                self.launch_command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=os.setsid  # Create new process group
            )
            
            # Give it a moment to start
            time.sleep(2)
            
            # Check if process is still running (didn't immediately fail)
            if self.process.poll() is None:
                self.status = ROSStatus.RUNNING
                self.logger.info("ROS launch process started successfully")
                return True
            else:
                # Process failed to start
                stdout, stderr = self.process.communicate()
                error_msg = f"Process failed to start. stderr: {stderr}"
                self.logger.error(error_msg)
                self.error_message = error_msg
                self.status = ROSStatus.ERROR
                return False
                
        except Exception as e:
            error_msg = f"Failed to start ROS launch: {str(e)}"
            self.logger.error(error_msg)
            self.error_message = error_msg
            self.status = ROSStatus.ERROR
            return False
    
    def stop_ros_launch(self) -> bool:
        """
        Stop the ROS launch process gracefully using pkill.
        
        Returns:
            bool: True if stopped successfully, False otherwise
        """
        try:
            self.status = ROSStatus.STOPPING
            self.error_message = None
            self.logger.info("Stopping ROS launch process...")
            
            # First, try to stop our managed process gracefully if it exists
            if self.process:
                try:
                    # Send SIGTERM to the process group
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    # Wait briefly for graceful shutdown
                    self.process.wait(timeout=3)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    # Process might already be gone, continue with pkill
                    pass
                except Exception as e:
                    # Log the error but continue with pkill approach
                    error_msg = f"Warning during process cleanup: {str(e)}"
                    self.logger.warning(error_msg)
                    self.error_message = error_msg
                
                self.process = None
            
            # Use pkill to stop all ROS processes more efficiently
            result = subprocess.run(
                ["pkill", "-f", self.ros_root],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            # pkill returns 0 if processes were found and killed, 1 if no processes found
            # Both are considered success for our purposes
            if result.returncode in [0, 1]:
                # Give processes time to terminate
                time.sleep(2)
                self.status = ROSStatus.NOT_RUNNING
                self.logger.info("ROS launch process stopped successfully")
                return True
            else:
                error_msg = f"pkill failed with return code {result.returncode}: {result.stderr}"
                self.logger.error(error_msg)
                self.error_message = error_msg
                self.status = ROSStatus.ERROR
                return False
                
        except subprocess.TimeoutExpired:
            error_msg = "Timeout while stopping ROS processes"
            self.logger.error(error_msg)
            self.error_message = error_msg
            self.status = ROSStatus.ERROR
            return False
        except Exception as e:
            error_msg = f"Failed to stop ROS launch: {str(e)}"
            self.logger.error(error_msg)
            self.error_message = error_msg
            self.status = ROSStatus.ERROR
            return False
    
    def is_running(self) -> bool:
        """
        Check if the ROS launch process is currently running.
        
        Returns:
            bool: True if process is running, False otherwise
        """
        if self.process is None:
            return False
        
        # Check if process is still alive
        poll_result = self.process.poll()
        if poll_result is None:
            # Process is still running
            self.status = ROSStatus.RUNNING
            return True
        else:
            # Process has terminated
            if poll_result != 0:
                # Process ended with error
                try:
                    stdout, stderr = self.process.communicate()
                    error_msg = f"Process terminated with code {poll_result}. stderr: {stderr}"
                    self.logger.error(error_msg)
                    self.error_message = error_msg
                except:
                    error_msg = f"Process terminated with code {poll_result}"
                    self.logger.error(error_msg)
                    self.error_message = error_msg
                self.status = ROSStatus.ERROR
            else:
                self.logger.info("ROS launch process terminated normally")
                self.status = ROSStatus.NOT_RUNNING
            
            self.process = None
            return False
    
    def get_status(self) -> Tuple[ROSStatus, Optional[str]]:
        """
        Get current status and any error message.
        
        Returns:
            Tuple[ROSStatus, Optional[str]]: Current status and error message if any
        """
        # Update status by checking if process is running
        self.is_running()
        return self.status, self.error_message
    
    def get_process_output(self) -> Tuple[Optional[str], Optional[str]]:
        """
        Get stdout and stderr from the process (non-blocking).
        
        Returns:
            Tuple[Optional[str], Optional[str]]: stdout and stderr content
        """
        if not self.process:
            return None, None
        
        try:
            # Non-blocking read
            stdout = self.process.stdout.read() if self.process.stdout else None
            stderr = self.process.stderr.read() if self.process.stderr else None
            return stdout, stderr
        except:
            return None, None
    
    def restart(self) -> bool:
        """
        Restart the ROS launch process.
        
        Returns:
            bool: True if restarted successfully, False otherwise
        """
        self.stop_ros_launch()
        time.sleep(2)  # Give it time to fully stop
        return self.start_ros_launch()

def get_ros_manager():
    """
    Dependency function to get a singleton ROSLaunchManager instance.
    This ensures we reuse the same manager instance across all requests.
    """
    if not hasattr(get_ros_manager, "_instance"):
        get_ros_manager._instance = ROSLaunchManager("wx250")
    return get_ros_manager._instance
