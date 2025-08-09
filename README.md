# Interbotix Control Server

```bash
emackinnon1@rpi4-widowx250:~/InterbotixControlServer$ ros2 service list

/wx250/get_motor_registers
/wx250/get_robot_info
/wx250/reboot_motors
/wx250/robot_state_publisher/describe_parameters
/wx250/robot_state_publisher/get_parameter_types
/wx250/robot_state_publisher/get_parameters
/wx250/robot_state_publisher/list_parameters
/wx250/robot_state_publisher/set_parameters
/wx250/robot_state_publisher/set_parameters_atomically
/wx250/rviz2/describe_parameters
/wx250/rviz2/get_parameter_types
/wx250/rviz2/get_parameters
/wx250/rviz2/list_parameters
/wx250/rviz2/set_parameters
/wx250/rviz2/set_parameters_atomically
/wx250/set_motor_pid_gains
/wx250/set_motor_registers
/wx250/set_operating_modes
/wx250/torque_enable
/wx250/xs_sdk/describe_parameters
/wx250/xs_sdk/get_parameter_types
/wx250/xs_sdk/get_parameters
/wx250/xs_sdk/list_parameters
/wx250/xs_sdk/set_parameters
/wx250/xs_sdk/set_parameters_atomically

```

```bash
emackinnon1@rpi4-widowx250:~/InterbotixControlServer$ ros2 node list
WARNING: Be aware that are nodes in the graph that share an exact name, this can have unintended side effects.
/wx250/robot_state_publisher
/wx250/rviz2
/wx250/rviz2
/wx250/transform_listener_impl_aaaacf2bb6e0
/wx250/xs_sdk
```

Will need to rebuild the interbotix_xsarm_control package if attempting to add Shutdown back on line 105 in `install/interbotix_xsarm_control/share/interbotix_xsarm_control/launch/xsarm_control.launch.py`
```python
from launch.actions import Shutdown

on_exit=Shutdown() # Try shutting down on exit
```

```bash
colcon build --packages-select interbotix_xsarm_control
# or
colcon build --paths interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control
colcon build --paths InterbotixControlServer/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control
```


Commands to remember:
```bash
# launch
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250 use_sim:=true
# run script
python3 /home/emackinnon1/InterbotixControlServer/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/scripts/open_beer.py
# disable/enable torque
ros2 service call /wx250/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
```

stopping fastapi when addr already in use
```bash
sudo lsof -t -i tcp:8000 | xargs kill -9
```

Run prod server:
```bash
uv run uvicorn main:app --host 0.0.0.0 --port 8000
```