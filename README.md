# Interbotix Control Server

In order to get this application up and running, the `xsarm_rpi4_install.sh` script should be run (a modified version of the one found [here](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/raspberry_pi_setup.html)). The install path will default to `InterbotixControlServer/workspace`.

Commands to launch ros and test python code:
```bash
# launch in sim mode
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250 use_sim:=true
# run open_beer script
python3 /scripts/open_beer.py
# disable/enable torque manually
ros2 service call /wx250/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
```

stopping fastapi when addr already in use
```bash
sudo lsof -t -i tcp:8000 | xargs kill -9

# or...
ps aux | grep uvicorn
kill -SIGINT <PID>
```

Run prod server:
```bash
uv run uvicorn main:app --host 0.0.0.0 --port 8000
```