# Interbotix Control Server

## Getting started
In order to get this application up and running, the `xsarm_rpi_install.sh` script should be run (a modified version of the one found [here](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/raspberry_pi_setup.html)). The install path will default to `InterbotixControlServer/workspace` and will install all necessary ros packages.

The FastAPI lifespan starts ROS, initializes the robot, and starts the arm movement worker automatically. Robot initialization endpoints remain available for compatibility but are not required before sending movement or state-machine commands. On shutdown, active execution is stopped, the arm is safely shut down, and ROS is stopped.

### Deployment and services
Run the service installer once as the service user:
```bash
./scripts/install-boot-service.sh
```

This installs and enables `interbotix-control.service`, configures the project Caddyfile for the package-managed `caddy.service`, and enables user lingering so FastAPI starts at boot. FastAPI runs under systemd with ROS Jazzy and the workspace environment sourced. Caddy stays up independently during application deployments and normal cleanup.

The GitHub Actions workflow runs tests for pull requests. A push to `main` updates `/home/emackinnon1/InterbotixControlServer` and deploys the application there. From that checkout, a deployment can also be run manually with:
```bash
make deploy
```

Check or restart FastAPI and inspect its journal:
```bash
systemctl --user status interbotix-control.service
systemctl --user restart interbotix-control.service
journalctl --user -u interbotix-control.service -f
```

Check Caddy separately:
```bash
sudo systemctl status caddy.service
sudo journalctl -u caddy.service -f
```

Stop FastAPI without stopping Caddy, or explicitly stop both:
```bash
make clean
bash scripts/cleanup.sh --include-caddy
```

### Cleaning ros build
If the ROS build enters a broken state, sometimes the easiest solution is to do a clean build. Delete the `/workspace` directory and remove the desktop package for your installed ROS distro (for example `ros-jazzy-desktop`):
```bash
sudo apt-get remove "ros-${ROS_DISTRO:-jazzy}-desktop" && sudo apt-get autoremove
```
Then, run the `xsarm_rpi_install.sh` again.


## Miscellanious info:
Commands to launch ros and test python code:
```bash
# launch in sim mode
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250 use_sim:=true
# run open_beer script
python3 /scripts/open_beer.py
# disable/enable torque manually
ros2 service call /wx250/torque_enable interbotix_xs_msgs/srv/TorqueEnable "{cmd_type: 'group', name: 'all', enable: true}"
```

If port 8000 is already in use, inspect the managed service rather than killing arbitrary processes:
```bash
systemctl --user status interbotix-control.service
journalctl --user -u interbotix-control.service -n 50
```

Run the server manually only when the systemd service is stopped:
```bash
source /opt/ros/jazzy/setup.bash
source workspace/install/setup.bash
.venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000
```