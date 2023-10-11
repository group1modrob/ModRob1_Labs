# Copyright 2022 Trossen Robotics
import numpy as np 
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():

    # Define the angles for each joint
    waist_j1=90*np.pi/180
    shoulder_j2=-45*np.pi/180
    elbow_j3=0*np.pi/180
    wrist_j4=45*np.pi/180

    # organize them into a list
    joint_positions = [waist_j1, shoulder_j2, elbow_j3, wrist_j4]

    # create the px100 object
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )

    # Send it to the desired position!
    bot.arm.go_to_home_pose()
    bot.arm.set_single_joint_position(joint_name='waist', position=waist_j1)
    print("here1!")
    bot.arm.set_single_joint_position(joint_name='shoulder', position=shoulder_j2)
    print("here2!")
    bot.arm.set_single_joint_position(joint_name='elbow', position=elbow_j3)
    print("here3!")
    bot.arm.set_single_joint_position(joint_name='wrist_angle', position=wrist_j4)
    print("here4!")

    # Go back home
    # bot.arm.go_to_home_pose()
    # bot.arm.go_to_sleep_pose()
    # bot.shutdown()


if __name__ == '__main__':
    main()