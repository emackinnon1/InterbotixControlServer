# create dependency to double check ROS status before running commands
import asyncio
import subprocess
import time
import signal
import os
import logging
from enum import Enum
from typing import Optional, Tuple, Union


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
        # self.logger = logging.getLogger("uvicorn")
        self.robot_model = robot_model
        self.process: Optional[Union[subprocess.Popen, asyncio.subprocess.Process]] = None
        self.status = ROSStatus.NOT_RUNNING
        self.error_message: Optional[str] = None
        self.streaming_task: Optional[asyncio.Task] = None  # Track streaming task
        self.launch_command = [
            "ros2",
            "launch", 
            "interbotix_xsarm_control", 
            "xsarm_control.launch.py", 
            f"robot_model:={robot_model}"
        ]
        self.ros_root = "/opt/ros/humble"
    
    async def _stream_process_output(self, process: asyncio.subprocess.Process, process_name: str):
        """
        Stream stdout and stderr from a process to logs in real-time.
        
        Args:
            process: The asyncio subprocess to stream from
            process_name: Name to use in log messages
        """
        async def stream_stdout():
            if process.stdout:
                async for line in process.stdout:
                    decoded_line = line.decode().strip()
                    if decoded_line:
                        self.logger.info(f"ROS {process_name} stdout: {decoded_line}")
        
        async def stream_stderr():
            if process.stderr:
                async for line in process.stderr:
                    decoded_line = line.decode().strip()
                    if decoded_line:
                        self.logger.warning(f"ROS {process_name} stderr: {decoded_line}")
        
        # Start both streaming tasks
        await asyncio.gather(
            stream_stdout(),
            stream_stderr(),
            return_exceptions=True
        )
    
    async def check_ros2_running(self) -> bool:
        """
        Check if ROS2 processes are currently running using ps aux.
        
        Returns:
            bool: True if ROS2 processes are found, False otherwise
        """
        try:
            # Use asyncio subprocess for non-blocking execution
            process = await asyncio.create_subprocess_exec(
                "ps", "aux",
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await asyncio.wait_for(process.communicate(), timeout=10)
            
            # Check for ROS-related processes
            grep_process = await asyncio.create_subprocess_exec(
                "grep", "-i", "ros",
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            grep_stdout, grep_stderr = await grep_process.communicate(input=stdout)
            
            # Filter out this script's own process
            filtered_output = []
            for line in grep_stdout.decode().splitlines():
                if "ros_manager.py" not in line and "grep -i ros" not in line:
                    filtered_output.append(line)
            
            return len(filtered_output) > 0
            
        except (asyncio.TimeoutError, Exception) as e:
            error_msg = f"Failed to check ROS status: {str(e)}"
            self.logger.error(error_msg)
            self.error_message = error_msg
            return False
    
    async def start_ros_launch(self) -> bool:
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
            
            # Start the process in the background using asyncio
            self.process = await asyncio.create_subprocess_exec(
                *self.launch_command,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group
            )
            
            # Give it a moment to start
            await asyncio.sleep(2)
            
            # Check if process is still running (didn't immediately fail)
            if self.process.returncode is None:
                self.status = ROSStatus.RUNNING
                self.logger.info("ROS launch process started successfully")
                
                # Start streaming output in the background
                self.streaming_task = asyncio.create_task(
                    self._stream_process_output(self.process, "launch")
                )
                
                return True
            else:
                # Process failed to start
                stdout, stderr = await self.process.communicate()
                error_msg = f"Process failed to start. stderr: {stderr.decode()}"
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
    
    async def stop_ros_launch(self) -> bool:
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
                # Cancel streaming task if it exists
                if self.streaming_task and not self.streaming_task.done():
                    self.streaming_task.cancel()
                    try:
                        await self.streaming_task
                    except asyncio.CancelledError:
                        pass
                    self.streaming_task = None
                
                try:
                    # Send SIGTERM to the process group
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    # Wait briefly for graceful shutdown
                    await asyncio.wait_for(self.process.wait(), timeout=3)
                except (asyncio.TimeoutError, ProcessLookupError):
                    # Process might already be gone, continue with pkill
                    pass
                except Exception as e:
                    # Log the error but continue with pkill approach
                    error_msg = f"Warning during process cleanup: {str(e)}"
                    self.logger.warning(error_msg)
                    self.error_message = error_msg
                
                self.process = None
            
            # Use pkill to stop all ROS processes more efficiently
            pkill_process = await asyncio.create_subprocess_exec(
                "pkill", "-f", self.ros_root,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            stdout, stderr = await asyncio.wait_for(pkill_process.communicate(), timeout=10)
            
            # pkill returns 0 if processes were found and killed, 1 if no processes found
            # Both are considered success for our purposes
            if pkill_process.returncode in [0, 1]:
                # Give processes time to terminate
                await asyncio.sleep(2)
                self.status = ROSStatus.NOT_RUNNING
                self.logger.info("ROS launch process stopped successfully")
                return True
            else:
                error_msg = f"pkill failed with return code {pkill_process.returncode}: {stderr.decode()}"
                self.logger.error(error_msg)
                self.error_message = error_msg
                self.status = ROSStatus.ERROR
                return False
                
        except asyncio.TimeoutError:
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
        
        # Check if process is still alive - handle both sync and async processes
        if isinstance(self.process, asyncio.subprocess.Process):
            poll_result = self.process.returncode
        else:
            poll_result = self.process.poll()
            
        if poll_result is None:
            # Process is still running
            self.status = ROSStatus.RUNNING
            return True
        else:
            # Process has terminated
            if poll_result != 0:
                # Process ended with error
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
    
    async def restart(self) -> bool:
        """
        Restart the ROS launch process.
        
        Returns:
            bool: True if restarted successfully, False otherwise
        """
        await self.stop_ros_launch()
        await asyncio.sleep(2)  # Give it time to fully stop
        return await self.start_ros_launch()

def get_ros_manager():
    """
    Dependency function to get a singleton ROSLaunchManager instance.
    This ensures we reuse the same manager instance across all requests.
    """
    if not hasattr(get_ros_manager, "_instance"):
        get_ros_manager._instance = ROSLaunchManager("wx250")
    return get_ros_manager._instance
