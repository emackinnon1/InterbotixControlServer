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

Will need to rebuild the interbotix_xsarm_control package if adding 
```python
on_exit=Shutdown() # Try shutting down on exit
```
back on line 105 in `install/interbotix_xsarm_control/share/interbotix_xsarm_control/launch/xsarm_control.launch.py`
```bash
colcon build --packages-select interbotix_xsarm_control
```