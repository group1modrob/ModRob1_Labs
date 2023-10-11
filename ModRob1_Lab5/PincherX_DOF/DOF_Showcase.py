# Imports
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
import time

def main():
    # Define the robot
    bot = InterbotixManipulatorXS(
            robot_model='px100',
            group_name='arm',
            gripper_name='gripper'
            )  
    
    sleep_time = 1.0

    # ROBOT CHOREOGRAPHY
    # First go to HOME
    bot.arm.go_to_home_pose()
    time.sleep(sleep_time)

    # Move the waist 90 degrees. The rotation limits for this joint are from -180 deg to 180 deg, so 90 deg is fine
    bot.arm.set_single_joint_position(joint_name='waist', position=np.pi/2.0)
    time.sleep(sleep_time)

    # Move it back to -90 degrees
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0)
    time.sleep(sleep_time)

    # Move it back to home
    bot.arm.go_to_home_pose()
    time.sleep(sleep_time)

    # Now move the SHOULDER to 20 degrees
    bot.arm.set_single_joint_position(joint_name='shoulder', position=20*np.pi/180)
    time.sleep(sleep_time)

    # Now move it to -20 degrees
    bot.arm.set_single_joint_position(joint_name='shoulder', position=-20*np.pi/180)
    time.sleep(sleep_time)

    # Now move it back to home
    bot.arm.go_to_home_pose()
    time.sleep(sleep_time)

    # Now move the ELBOW to 45 degrees
    bot.arm.set_single_joint_position(joint_name='elbow', position=45*np.pi/180)
    time.sleep(sleep_time)

    # Now move the ELBOW to -45 degrees
    bot.arm.set_single_joint_position(joint_name='elbow', position=-45*np.pi/180)
    time.sleep(sleep_time)

    # Now move it back to home
    bot.arm.go_to_home_pose()
    time.sleep(sleep_time)

    # Now move the WRIST to 90 degrees
    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=90*np.pi/180)
    time.sleep(sleep_time)

    # Now move the WRIST to -90 degrees
    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=-90*np.pi/180)
    time.sleep(sleep_time)

    # Now move it back to home
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()

