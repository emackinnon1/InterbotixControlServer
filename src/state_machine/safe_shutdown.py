import time

def safe_shutdown_sync(bot: 'InterbotixManipulatorXS') -> bool:
    """Safely release gripper, go home, go sleep, disable torque.
    Returns True if torque disable step succeeds; False otherwise.
    Any intermediate errors are logged but do not abort sequence.
    """
    try:
        bot.gripper.release()
        print("[safe_shutdown] Gripper released")
        time.sleep(0.5)
    except Exception as e:
        print(f"[safe_shutdown] Warning release gripper: {e}")
    try:
        bot.arm.go_to_home_pose(moving_time=2.0)
        print("[safe_shutdown] Home pose reached")
        time.sleep(1.0)
    except Exception as e:
        print(f"[safe_shutdown] Warning home pose: {e}")
    try:
        bot.arm.go_to_sleep_pose(moving_time=2.0)
        print("[safe_shutdown] Sleep pose reached")
        time.sleep(1.0)
    except Exception as e:
        print(f"[safe_shutdown] Warning sleep pose: {e}")
    try:
        bot.core.robot_reboot_motors(cmd_type='group', name='all', enable=False, smart_reboot=True)
        bot.core.robot_torque_enable(cmd_type='group', name='all', enable=False)
        print("[safe_shutdown] Torque disabled")
        return True
    except Exception as e:
        print(f"[safe_shutdown] Warning torque disable: {e}")
        return False
