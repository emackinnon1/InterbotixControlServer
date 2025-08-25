#!/usr/bin/env python3
import time
from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

BOTTLE_LOWER_DISTANCE = -0.2

# can add the unique "open_bottle" steps if more are needed depending on size of
# other brands
BRAND_CONFIGS = {
    "sapporo": {
        "waist_rotation": -np.pi/5,
        "bottle_lower_distance": BOTTLE_LOWER_DISTANCE
    },
    "heineken": {
        "waist_rotation": -np.pi/4.75,
        "bottle_lower_distance": BOTTLE_LOWER_DISTANCE
    },
    "corona": {
        "waist_rotation": -np.pi/4.75,
        "bottle_lower_distance": -0.182
    }
}


def bottle_opener(bot: InterbotixManipulatorXS, put_back: bool):
    if not put_back:
        bot.gripper.release()
    bot.arm.go_to_home_pose(moving_time=1.75)
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


def open_bottle(bot: InterbotixManipulatorXS, brand: str):
    if not brand:
        print("BEER BRAND REQUIRED. REPLACING BOTTLE OPENER.")
        return
    bot.arm.go_to_home_pose()
    # bot.gripper.grasp()
    bot.arm.set_ee_pose_components(x=0.21, z=0.35)
    print("open bottle 1: inital pose")
    time.sleep(2.5)
    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=np.pi/3.5)
    print("open bottle 2: rotate wrist")
    time.sleep(2.5)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/5.65)
    print("open bottle 3: rotate waist")
    time.sleep(2.5)

    bottle_lower_distance = BRAND_CONFIGS.get(brand, {}).get("bottle_lower_distance")
    bot.arm.set_ee_cartesian_trajectory(z=bottle_lower_distance)
    print("open bottle 4: lower end effector")
    time.sleep(5)
    # get correct waist rotation
    waist_rotation_config = BRAND_CONFIGS.get(brand, {}).get("waist_rotation")
    bot.arm.set_single_joint_position(joint_name='waist', position=waist_rotation_config)
    
    print("open bottle 5: bottle opener in place")
    time.sleep(2.5)
    bot.arm.set_single_joint_position(joint_name='wrist_rotate', position=np.pi/1.2, moving_time=0.5)
    bot.arm.set_ee_cartesian_trajectory(z=0.1)
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/5.65, moving_time=1.5)
    print("open bottle 6: raise, rotatewrist and open")
    bot.arm.go_to_home_pose(moving_time=1.75)


def open_beer(brand: str):

    bot = InterbotixManipulatorXS(
        robot_model='wx250',
        group_name='arm',
        gripper_name='gripper',
        moving_time=1.5,
        gripper_pressure=0.85           
    )

    robot_startup()
    # bot.gripper.set_pressure(1.0)
    bot.core.robot_torque_enable(cmd_type='group', name='all', enable=True)
    

    # bot.arm.go_to_home_pose()
    bottle_opener(bot, False)
    open_bottle(bot, brand)
    bottle_opener(bot, True)
    
    bot.arm.go_to_home_pose()

    bot.arm.go_to_sleep_pose()
    time.sleep(0.5)
    bot.core.robot_torque_enable(cmd_type='group', name='all', enable=False)
    robot_shutdown()


if __name__ == '__main__':
    open_beer("corona")