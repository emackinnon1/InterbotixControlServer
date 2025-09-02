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
        self.robot_model = robot_model
        self.process: Optional[Union[subprocess.Popen, asyncio.subprocess.Process]] = None
        self.status = ROSStatus.NOT_RUNNING
        self.error_message: Optional[str] = None
        self.streaming_task: Optional[asyncio.Task] = None  # Track streaming task
        self.monitoring_task: Optional[asyncio.Task] = None  # Track monitoring task
        self.monitoring_enabled: bool = False  # Control monitoring
        self.last_known_status: ROSStatus = ROSStatus.NOT_RUNNING  # Track status changes
        self.auto_restart_enabled: bool = True  # Enable automatic restart on failure
        self.critical_error_patterns = [
            "FATAL", "ERROR", "FAILED", "ABORT", "CRITICAL", 
            "SEGMENTATION FAULT", "CORE DUMPED", "EXCEPTION"
        ]  # Patterns that indicate critical errors
        self.launch_command = [
            "ros2",
            "launch", 
            "interbotix_xsarm_control", 
            "xsarm_control.launch.py", 
            f"robot_model:={robot_model}"
        ]
        self.ros_root = "/opt/ros/humble"
        self.recent_log_lines: list[str] = []  # Store recent log lines for error checking
        self.max_recent_log_lines: int = 100  # Limit for stored log lines
    
    def _is_critical_error(self, stderr_line: str) -> bool:
        """
        Check if a stderr line contains critical error patterns.
        
        Args:
            stderr_line: The stderr line to analyze
            
        Returns:
            bool: True if the line contains a critical error pattern
        """
        stderr_upper = stderr_line.upper()
        return any(pattern in stderr_upper for pattern in self.critical_error_patterns)
    
    def _add_log_line(self, line: str):
        """
        Add a log line to the recent log buffer, maintaining the max limit.
        """
        self.recent_log_lines.append(line)
        if len(self.recent_log_lines) > self.max_recent_log_lines:
            self.recent_log_lines = self.recent_log_lines[-self.max_recent_log_lines:]

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
                        self._add_log_line(decoded_line)
        
        async def stream_stderr():
            if process.stderr:
                async for line in process.stderr:
                    decoded_line = line.decode().strip()
                    if decoded_line:
                        self.logger.warning(f"ROS {process_name} stderr: {decoded_line}")
                        self._add_log_line(decoded_line)
                        
                        # Check for critical errors and update status immediately
                        if self._is_critical_error(decoded_line):
                            self.logger.error(f"Critical error detected in ROS output: {decoded_line}")
                            self.status = ROSStatus.ERROR
                            self.error_message = f"Critical error from stderr: {decoded_line}"
        
        # Start both streaming tasks
        await asyncio.gather(
            stream_stdout(),
            stream_stderr(),
            return_exceptions=True
        )
    
    async def _monitor_process_health(self):
        """
        Background task to monitor ROS process health and log status changes.
        Runs periodically while monitoring is enabled.
        """
        self.logger.info("Started ROS process health monitoring")
        
        while self.monitoring_enabled:
            try:
                # Check current process status
                current_running = self.is_running()
                current_status = self.status
                
                # Log status changes
                if current_status != self.last_known_status:
                    self.logger.info(f"ROS status changed from {self.last_known_status.value} to {current_status.value}")
                    
                    # Check if process failed unexpectedly and auto-restart is enabled
                    if (current_status == ROSStatus.ERROR and 
                        self.last_known_status == ROSStatus.RUNNING and
                        self.auto_restart_enabled):
                        self.logger.warning("ROS process failed unexpectedly, attempting automatic restart...")
                        
                        # Attempt to restart the process
                        try:
                            restart_success = await self.restart()
                            if restart_success:
                                self.logger.info("Automatic restart successful")
                            else:
                                self.logger.error("Automatic restart failed")
                        except Exception as restart_error:
                            self.logger.error(f"Error during automatic restart: {str(restart_error)}")
                    
                    self.last_known_status = current_status
                
                # Wait before next health check (every 5 seconds)
                await asyncio.sleep(5)
                
            except Exception as e:
                self.logger.error(f"Error during process health monitoring: {str(e)}")
                await asyncio.sleep(5)  # Continue monitoring even after errors
        
        self.logger.info("Stopped ROS process health monitoring")
    
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
                
                # Start health monitoring in the background
                self.monitoring_enabled = True
                self.last_known_status = ROSStatus.RUNNING
                self.monitoring_task = asyncio.create_task(
                    self._monitor_process_health()
                )
                
                # Monitor logs for errors after starting
                error_line = await self.monitor_logs_for_errors(timeout=5)
                if error_line:
                    self.logger.error(f"ROS launch error detected after start: {error_line}")
                    return False
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
                # Stop health monitoring
                self.monitoring_enabled = False
                if self.monitoring_task and not self.monitoring_task.done():
                    self.monitoring_task.cancel()
                    try:
                        await self.monitoring_task
                    except asyncio.CancelledError:
                        pass
                    self.monitoring_task = None
                
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
    
    async def monitor_logs_for_errors(self, timeout: int = 5) -> Optional[str]:
        """
        Monitor recent logs for specific error messages after start/restart.
        
        Args:
            timeout: seconds to wait and check logs
        
        Returns:
            str: Error message if found, else None
        """
        error_patterns = [
            "stack smashing detected",
            "process has died",
            "exit code -6"
        ]
        ignore_patterns = [
            "rviz2"
        ]
        import time
        start_time = time.time()
        while time.time() - start_time < timeout:
            for line in self.recent_log_lines:
                # Ignore rviz2 errors
                if any(ignore in line for ignore in ignore_patterns):
                    continue
                for pattern in error_patterns:
                    if pattern in line:
                        self.logger.error(f"Detected ROS launch error: {line}")
                        self.status = ROSStatus.ERROR
                        self.error_message = f"Detected error in logs: {line}"
                        return line
            await asyncio.sleep(0.5)
        return None

    def check_recent_logs_for_errors(self) -> Optional[str]:
        """
        Check recent logs for error patterns (non-async, for status endpoint or polling).
        
        Returns:
            str: Error message if found, else None
        """
        error_patterns = [
            "stack smashing detected",
            "process has died",
            "exit code -6"
        ]
        ignore_patterns = [
            "rviz2"
        ]
        for line in self.recent_log_lines:
            if any(ignore in line for ignore in ignore_patterns):
                continue
            for pattern in error_patterns:
                if pattern in line:
                    return line
        return None
    
    async def restart(self) -> bool:
        """
        Restart the ROS launch process.
        
        Returns:
            bool: True if restarted successfully, False otherwise
        """
        await self.stop_ros_launch()
        await asyncio.sleep(2)  # Give it time to fully stop
        result = await self.start_ros_launch()
        # Monitor logs for errors after restart
        if result:
            error_line = await self.monitor_logs_for_errors(timeout=5)
            if error_line:
                self.logger.error(f"ROS launch error detected after restart: {error_line}")
                return False
        return result

def get_ros_manager():
    """
    Dependency function to get a singleton ROSLaunchManager instance.
    This ensures we reuse the same manager instance across all requests.
    """
    if not hasattr(get_ros_manager, "_instance"):
        get_ros_manager._instance = ROSLaunchManager("wx250")
    return get_ros_manager._instance
