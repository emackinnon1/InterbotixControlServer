## 8-16
Dockerfile is now able to launch successfully.

docker build -t interbotix_base .

docker run -it --privileged --network=host --device=/dev/ttyUSB0 --device=/dev/ttyACM0 interbotix_base /bin/bash

docker exec interbotix-server ls -la /InterbotixControlServer/install/interbotix_xs_sdk/lib/interbotix_xs_sdk/xs_sdk

ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=wx250


# Next prompt
When I build and then running this docker container, I still keep getting this error:
```
src.dependencies.ros_manager - INFO - ROS launch stdout: [xs_sdk-1] [INFO] [1756056543.860615] Pinging all motors specified in the motor_config file. (Attempt 1/3)
```
which gets followed by:
```
[xs_sdk-1] [INFO] [1755962166.485912] Pinging all motors specified in the motor_config file. (Attempt 1/3)
[ERROR] [xs_sdk-1]: process has died [pid 240, exit code -11, cmd '/InterbotixControlServer/install/interbotix_xs_sdk/lib/interbotix_xs_sdk/xs_sdk --ros-args -r __node:=xs_sdk -r __ns:=/wx250 --params-file /tmp/launch_params_1bh2wamy'].
```
I want to figure out why building this container results in this error, but when I run the app locally without the container, the `xs_sdk` process works fine. The docker build logs are in `docker-build.log` and show an stderr for dynamixel_workbench_toolbox. I believe it is a problem with communicating with the motors which could be from a package that is not building or it could be from a difference in the overall build process with Docker. Help me debug this and let's figure out what the difference between running this locally versus in the docker container is.