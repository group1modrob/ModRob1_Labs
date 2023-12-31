# Copyright 2022 Trossen Robotics
import numpy as np 

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

def main():

    # TODO: Define the joint angles in radians considering the joint limits
    
    #joint limts in degrees
    waist_j1=0
    shoulder_j2=30
    elbow_j3=10
    wrist_j4=0

    #Conditionals to secure max and min lmit
    if waist_j1 >=180:
        waist_j1=175
        print("Max Reached, Joint 1 limit is 175 degrees")
    elif waist_j1 <=-180:
        waist_j1=-175
        print("Min Reached, Joint 1 limit is -175 degrees")

    if shoulder_j2 >=107:
        shoulder_j2=100
        print("Max Reached, Joint 2 limit is 100 degrees")
    elif shoulder_j2 <=-111:
        shoulder_j2=-100
        print("Min Reached, Joint 2 limit is -100 degrees")
    
    if elbow_j3 >=92:
        elbow_j3=85
        print("Max Reached, Joint 3 limit is 85 degrees")
    elif elbow_j3 <=-121:
        print("Max Reached, Joint 3 limit is -115 degrees")
        waist_j1=-115

    if wrist_j4 >=123:
        wrist_j4=115
        print("Max Reached, Joint 4 limit is 115 degrees")
    elif wrist_j4 <=-100:
        wrist_j4=-90
        print("Max Reached, Joint 4 limit is -90 degrees")

    waist_j1=waist_j1*np.pi/180
    shoulder_j2=shoulder_j2*np.pi/180
    elbow_j3=elbow_j3*np.pi/180
    wrist_j4=wrist_j4*np.pi/180

    joint_positions = [waist_j1, shoulder_j2, elbow_j3, wrist_j4]

    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    bot.arm.go_to_home_pose()
    bot.gripper.release(1.0)
    bot.arm.set_joint_positions(joint_positions)
    bot.gripper.grasp(2.0)
    bot.gripper.set_pressure(0.5)
    bot.arm.go_to_home_pose()
    bot.arm.set_single_joint_position(joint_name='waist', position=75*np.pi/180)
    bot.arm.set_ee_cartesian_trajectory(z=-0.1)
    bot.gripper.release(1.0)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

    bot.shutdown()


if __name__ == '__main__':
    main()