#!/usr/bin/env python3

# Copyright 2024 Trossen Robotics
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

"""
This script commands some arbitrary positions to the arm joints:

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch robot_model:=wx250s

Then change to this directory and type:

    python3 joint_position_control.py
"""


def main():
    joint_positions = [0, 0.5, 0.5, 0, -1.5]
    joint_positions_2 = [0, 0.9, 1.1, -0.4, -1.5]
    joint_positions_4 = [0, 1.17, 0.93, -0.53, -1.5]
    joint_positions_3 = [0, 0.75, 1.2, -0.3, -1.5]


    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()

    bot.arm.go_to_home_pose()
    bot.gripper.set_pressure(1.0)
    bot.gripper.release()
    bot.arm.set_joint_positions(joint_positions)
    bot.arm.set_joint_positions(joint_positions_2)
    # bot.arm.set_joint_positions(joint_positions_3)
    bot.arm.set_joint_positions(joint_positions_4)
    bot.gripper.grasp()
    bot.arm.set_joint_positions(joint_positions_2)
    bot.arm.set_joint_positions(joint_positions_3)
    bot.arm.set_joint_positions(joint_positions)

    bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()

    robot_shutdown()


if __name__ == '__main__':
    main()
