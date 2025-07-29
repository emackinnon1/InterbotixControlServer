#!/usr/bin/env python3
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np



def bottle_opener(bot, put_back):
    if not put_back:
        bot.gripper.release()
    bot.arm.go_to_home_pose(moving_time=2.0)
    # time.sleep(2.0)
    # bot.arm.set_ee_pose_components(x=0.268, z=0.23)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/1.89)
    print("pos 1: rotate")
    # time.sleep(3.0)
    bot.arm.set_ee_cartesian_trajectory(z=-0.15, x=-0.193)
    print("pos 2: lower")
    time.sleep(5)
    bot.arm.set_ee_cartesian_trajectory(x=-0.193)
    print("pos 3: reverse")
    time.sleep(5)
    bot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    print("pos 4: wrist pitch")
    time.sleep(3.5)
    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=np.pi/2.11)
    print("pos 5: wrist rotate")
    time.sleep(2.0)
    bot.arm.set_ee_cartesian_trajectory(z=-0.16) # gripper in position
    print("pos 6: lower end effector")
    time.sleep(5.0)
    if put_back:
        bot.gripper.release()
        print("pos 7: release gripper")
    else:
        bot.gripper.grasp()
        print("pos 7: close gripper")
    bot.arm.set_ee_cartesian_trajectory(z=0.19)
    print("pos 7: raise end effector")
    time.sleep(3.0)
    # bot.arm.go_to_home_pose()


def open_bottle(bot):
    bot.arm.go_to_home_pose()
    # bot.gripper.grasp()
    bot.arm.set_ee_pose_components(x=0.21, z=0.35)
    print("open bottle 1: inital pose")
    time.sleep(3)
    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=np.pi/3.5)
    print("open bottle 2: rotate wrist")
    time.sleep(3)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/5.65)
    print("open bottle 3: rotate waist")
    time.sleep(3)
    bot.arm.set_ee_cartesian_trajectory(z=-0.2)
    print("open bottle 4: lower end effector")
    time.sleep(5)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/5)
    print("open bottle 5: bottle opener in place")
    time.sleep(3)
    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=np.pi/1.2, moving_time=0.5)
    bot.arm.set_ee_cartesian_trajectory(z=0.1)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/5.65, moving_time=1.5)
    print("open bottle 6: raise, rotatewrist and open")
    bot.arm.go_to_home_pose(moving_time=2.0)




def main():

    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
        moving_time=1.5,
        gripper_pressure=0.85           
    )

    robot_startup()
    # bot.gripper.set_pressure(1.0)
    

    # bot.arm.go_to_home_pose()
    bottle_opener(bot, False)
    open_bottle(bot)
    bottle_opener(bot, True)
    
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    robot_shutdown()


if __name__ == '__main__':
    main()
