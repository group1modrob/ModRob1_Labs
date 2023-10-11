# Copyright 2022 Trossen Robotics 

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np

def main():
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    #Instructs the robot to go to home position
    bot.arm.go_to_home_pose()

    # Here we give the desired orientation to the waist. The joint position is in radians. We imported numpy library because we wanted to
    # use the the mathematical constant π 
    # Note that you can substitute the waist with any of the joints of the robot 

    # Using the set_ee_cartesian_trajectory() function from the arm.py library, you can define a linear trajectory using a series of 
    # waypoints that the end-effector should follow as it travels from its current pose to the desired pose
    #Instructions the robot to then go to a specific position in the negative space of the z axis
    #This is where the block is located in the z axis in meters
    #Instructs the robot to go down 16cm  in the z direction 
    bot.arm.set_ee_cartesian_trajectory(z=-0.16)
   
    # grasp() and release() functions from gripper.py library to control the gripper
    # TODO: go to the gripper.py library.
    # Instructs the robot to grasp for 1 second   
    bot.gripper.grasp(1.0)
    # Defines the pressure at which the pincher grabs the block from a scale factor from 0 to 1
    bot.gripper.set_pressure(0.50)
    #Instructs the robot to go up 10cm  in the z direction 
    bot.arm.set_ee_cartesian_trajectory(z=0.1)
    
    # InterbotixGripperXSInterface class
    # Instructs the robot to rotate 90 degrees around the x axis from the intial position
    bot.arm.set_single_joint_position(joint_name='waist', position=-np.pi/2.0)
    #Instructs robot to move down 10 cm in the z position to place the block on the ground
    bot.arm.set_ee_cartesian_trajectory(z=-0.1)
    # Releases the robot pincher to drop the block and then waits 2 seconds   
    bot.gripper.release(2.0)
    
    #Instructs robot to move up 10 cm in the z position to place the block on the ground
    bot.arm.set_ee_cartesian_trajectory(z=0.1)
    #Robot goes back to the home position
    bot.arm.go_to_home_pose()
    #Robot goes to sleep position
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()
