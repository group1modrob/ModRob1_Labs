from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from math import atan2, sqrt, pi, acos, sin, cos, asin
from scipy.linalg import logm, expm
import numpy as np
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from . import px100_IK_ex
from math import atan2, sin, cos, pi

"""
You might need to modify the following YAML files to change the speed of the robot's movement!
File #1: /home/group1/interbotix_ws/install/interbotix_xsarm_control/share/interbotix_xsarm_control/config/modes.yaml
File #2: /home/group1/interbotix_ws/src/interbotix_ros_manipulators/interbotix_ros_xsarms/interbotix_xsarm_control/config/modes.yaml

The parameter that you need to change to change the speed of the robot's movement is "profile_velocity" in File #2. The GREATER this number, the SLOWER your robot will move!

The code we used to obtain the R's and p's is part1.py from Lab 10
"""

# Start by defining some constants such as robot model, visual perception frames, basket transform, etc.
ROBOT_MODEL = 'px100'
ROBOT_NAME = ROBOT_MODEL
REF_FRAME = 'camera_color_optical_frame'
ARM_TAG_FRAME = f'{ROBOT_NAME}/ar_tag_link'
ARM_BASE_FRAME = f'{ROBOT_NAME}/base_link'

# Release Location will be the robot's HOME position! Only modify this if you want a different position for release
# Td_release = np.array([[0, -1, 0, 0],
#                        [1, 0, 0, 0.2426],
#                        [0, 0, 1, 0.18945],
#                        [0, 0, 0, 1]]) # Basket (or the appropriate object in your design) end-effector transformation

    
def main():
    # Initialize the arm module along with the point cloud, armtag modules and px100_IK_ex custom API
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        robot_name=ROBOT_NAME,
        group_name='arm',
        gripper_name='gripper'
    )
    pcl = InterbotixPointCloudInterface(node_inf=bot.core)
    armtag = InterbotixArmTagInterface(
        ref_frame=REF_FRAME,
        arm_tag_frame=ARM_TAG_FRAME,
        arm_base_frame=ARM_BASE_FRAME,
        node_inf=bot.core
    )
    my_api = px100_IK_ex.ourAPI()


    # set initial arm and gripper pose
    bot.arm.go_to_sleep_pose()
    bot.gripper.release()

    # get the ArmTag pose
    armtag.find_ref_to_arm_base_transform() 

    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the ARM_BASE_FRAME
    success, clusters = pcl.get_cluster_positions(
        ref_frame=ARM_BASE_FRAME,
        sort_axis='x',
        reverse=True
    )

    # Define a range for blue, red, and yellow colors. We will be picking up blue, red, and yellow blocks!
    # Change this according to the colored blocks you will pick up. These are RGB value ranges!
    lower_blue = (0, 10, 30)
    upper_blue = (60, 80, 100)

    lower_yellow = (120, 100, 20)
    upper_yellow = (170, 160, 70)

    lower_red = (100, 0, 0)
    upper_red = (150, 70, 70)

    # Z offset required
    z_offset = 0.02

    # Block counter to stop the robot from picking up unnecessary blocks
    block_counter = 0

    # The algorithm!
    if success:
        # Start off by going to HOME pose
        bot.arm.go_to_home_pose()

        # Define the color of the block to be picked up, take it to HOME basket, and drop it!
        # Ignore all other detected clusters!
        for cluster in clusters:
            
            if all(lower_blue[i] <= cluster['color'][i] <= upper_blue[i] for i in range(3)):
                print("\n\nPICKING UP BLUE...\n\n")
                
                # Increase the block counter
                block_counter = block_counter + 1
                
                # Get the first cube location
                x, y, z = cluster['position']
                z = z - z_offset # Fingers link offset
                
                # Determine the angle to move the waist BEFORE moving the gripper
                theta_base = atan2(y,x) # Waist angle based on X and Y location of cube taken from camera
                new_x = x/cos(theta_base)-0.01

                # The GRASP location for the block
                Td_grasp = np.array([[0.6830127, -0.25881905,  0.6830127,  new_x],
                                     [0.1830127,  0.96592583,  0.1830127,  y],
                                     [-0.70710678,  0,  0.70710678,  z],
                                     [0,  0,  0,  1]])

                # Obtain the joint positions from the Inverse Kinematics Numeric solution from the previous project
                joint_positions = my_api.num_IK(Td_grasp, np.array([0,0,0,0]))
                
                # Move the robot waist first, THEN move the gripper down to the block, THEN continue with the rest of the choreography
                bot.arm.set_joint_positions([theta_base,0,0,0]) # Set waist position
                bot.arm.set_joint_positions(np.append(theta_base,joint_positions[1:])) # Move gripper to block
                bot.gripper.grasp(2.0) # Grasp block and wait 2 seconds 
                bot.arm.set_joint_positions([theta_base,0,0,0]) # Move arm UP but still in the same waist position
                bot.arm.go_to_home_pose() # Go to drop location
                bot.gripper.release(2.0) # Release and wait 1 second
            
            elif all(lower_yellow[i] <= cluster['color'][i] <= upper_yellow[i] for i in range(3)):
                print("\n\nPICKING UP YELLOW...\n\n")

                # Increase the block counter
                block_counter = block_counter + 1

                # Get the first cube location
                x, y, z = cluster['position']
                z = z - z_offset # Fingers link offset 

                # Determine the angle to move the waist BEFORE moving the gripper
                theta_base = atan2(y,x)  # Waist angle based on X and Y location of cube taken from camera
                new_x = x/cos(theta_base)-0.01 
                
                # The GRASP location for the block
                Td_grasp = np.array([[0.6830127, -0.25881905,  0.6830127,  new_x],
                                     [0.1830127,  0.96592583,  0.1830127,  y],
                                     [-0.70710678,  0,  0.70710678,  z],
                                     [0,  0,  0,  1]])

                # Obtain the joint positions from the Inverse Kinematics Numeric solution from the previous project
                joint_positions = my_api.num_IK(Td_grasp, np.array([0,0,0,0])) 
                
                # Move the robot waist first, THEN move the gripper down to the block, THEN continue with the rest of the choreography 
                bot.arm.set_joint_positions([theta_base,0,0,0]) # Set waist position
                bot.arm.set_joint_positions(np.append(theta_base,joint_positions[1:])) # Move gripper to block 
                bot.gripper.grasp(2.0) # Grasp block and wait 2 seconds 
                bot.arm.set_joint_positions([theta_base,0,0,0]) # Move arm UP but still in the same waist position
                bot.arm.go_to_home_pose()  #Go to drop location   
                bot.gripper.release(2.0) # Release and wait 1 second
            
            elif all(lower_red[i] <= cluster['color'][i] <= upper_red[i] for i in range(3)):
                print("\n\nPICKING UP RED...\n\n")

                # Increase the block counter
                block_counter = block_counter + 1
                print(cluster)
                
                # Get the first cube location
                x, y, z = cluster['position']
                z = z - z_offset # Fingers link offset 

                # Go on top of the selected cube
                theta_base = atan2(y,x) # Waist angle offset
                new_x = x/cos(theta_base)-0.01 
                
                # The GRASP location for the block
                Td_grasp = np.array([[0.6830127, -0.25881905,  0.6830127,  new_x],
                                     [0.1830127,  0.96592583,  0.1830127,  y],
                                     [-0.70710678,  0,  0.70710678,  z],
                                     [0,  0,  0,  1]])

                # Obtain the joint positions from the Inverse Kinematics Numeric solution from the previous project
                joint_positions = my_api.num_IK(Td_grasp, np.array([0,0,0,0])) 
                
                # Move the robot waist first, THEN move the gripper down to the block, THEN continue with the rest of the choreography  
                bot.arm.set_joint_positions([theta_base,0,0,0]) # Set waist position
                bot.arm.set_joint_positions(np.append(theta_base,joint_positions[1:])) # Move gripper to block 
                bot.gripper.grasp(2.0) # Grasp block and wait 2 seconds 
                bot.arm.set_joint_positions([theta_base,0,0,0]) # Move arm UP but still in the same waist position
                bot.arm.go_to_home_pose()  #Go to drop location      
                bot.gripper.release(2.0) # Release and wait 1 second
            
            if block_counter >= 3:
                break
    else:
        print('Could not get cluster positions.')

    # Go to sleep
    bot.arm.go_to_sleep_pose()
    bot.shutdown()


if __name__ == '__main__':
    main()