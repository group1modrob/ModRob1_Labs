from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from math import atan2, sqrt, pi, acos, sin, cos, asin
from scipy.linalg import logm, expm
import numpy as np

class ourAPI:
    def __init__(self):
        # Robot parameters
        self.L1 = 0.08945
        self.L2 = 0.100
        self.Lm =  0.035
        self.L3 = 0.100
        self.L4 = 0.1076
        self.S = np.array([[0, 0, 1, 0, 0, 0],
                            [0, 1, 0, -self.L1, 0, 0],
                            [0, 1, 0, -(self.L1+self.L2), 0, self.Lm],
                            [0, 1, 0, -(self.L1+self.L2), 0, self.Lm+self.L3]]) # Screw axes
        self.M = np.array([[1, 0, 0, self.Lm+self.L3+self.L4],
                            [0, 1, 0, 0],
                            [0, 0, 1, self.L1+self.L2],
                            [0, 0, 0, 1]]) # End-effector M matrix
    
    def screw_axis_to_transformation_matrix(self, screw_axis, angle):
        """
        Convert a screw axis and angle to a homogeneous transformation matrix.

        Parameters:
        - screw_axis: A 6D screw axis [Sw, Sv], where Sw is the rotational component
                        and Sv is the translational component.
        - angle: The angle of rotation in radians.

        Returns:
        - transformation_matrix: The 4x4 homogeneous transformation matrix
                                corresponding to the input screw axis and angle.
        """
        assert len(screw_axis) == 6, "Input screw axis must have six components"

        # Extract rotational and translational components from the screw axis
        Sw = screw_axis[:3]
        Sv = screw_axis[3:]

        # Matrix form of the screw axis
        screw_matrix = np.zeros((4, 4))
        screw_matrix[:3, :3] = np.array([[ 0, -Sw[2], Sw[1]],
                                         [Sw[2], 0, -Sw[0]],
                                         [-Sw[1], Sw[0], 0]])
        screw_matrix[:3, 3] = Sv

        # Exponential map to get the transformation matrix
        exponential_map = expm(angle * screw_matrix)
        
        return exponential_map
    
    def twist_vector_from_twist_matrix(self, twist_matrix):
        """
        Compute the original 6D twist vector from a 4x4 twist matrix.

        Parameters:
        - twist_matrix: A 4x4 matrix representing the matrix form of the twist 

        Returns:
        - twist_vector: The 6D twist vector [w, v] corresponding to the input
                        twist matrix.
        """
        assert twist_matrix.shape == (4, 4), "Input matrix must be 4x4"

        w = [twist_matrix[2, 1], twist_matrix[0, 2], twist_matrix[1, 0]]
        v = twist_matrix[:3, 3]

        return np.concatenate((w, v))

    def body_jacobian(self, angles):
        # The exponential form of each Screw Axis
        expS1 = self.screw_axis_to_transformation_matrix(self.S[0], angles[0])
        expS2 = self.screw_axis_to_transformation_matrix(self.S[1], angles[1])
        expS3 = self.screw_axis_to_transformation_matrix(self.S[2], angles[2])
        expS4 = self.screw_axis_to_transformation_matrix(self.S[3], angles[3])
        
        # Obtain Tsb and Tbs using the multiplication of the exponentials
        Tsb = expS1 @ expS2 @ expS3 @ expS4 @ self.M
        Tbs = np.linalg.inv(Tsb)

        # Obtain the adjoint of Tbs
        R_for_adj = np.array(Tbs[:3, :3])
        p_for_adj = np.array(Tbs[:3, 3])
        p_for_adj_bracket = np.array([[0, -p_for_adj[2], p_for_adj[1]], 
                                      [p_for_adj[2], 0, -p_for_adj[0]], 
                                      [-p_for_adj[1], p_for_adj[0], 0]])
        zero_matrix = np.zeros((3,3))
        Tbs_adj_first_part = np.hstack((R_for_adj, zero_matrix))
        Tbs_adj_second_part = np.hstack((p_for_adj_bracket @ R_for_adj, R_for_adj))
        Tbs_adj = np.vstack((Tbs_adj_first_part, Tbs_adj_second_part))

        # Obtain the SPACE Jacobian
        Js = np.array([[0, -np.sin(angles[0]), 0, 0],
                       [0, np.cos(angles[0]), 1, 1],
                       [1, 0, 0, 0],
                       [0, -0.08945*np.cos(angles[0]), 0.035*np.sin(angles[1])-0.1*np.cos(angles[1]) - 0.08945, 0.1*np.sin(angles[2])-0.18945],
                       [0, -0.08945*np.sin(angles[0]), 0, 0],
                       [0, 0, 0.035*np.cos(angles[1])+0.1*np.sin(angles[1]), 0.1*np.cos(angles[2]+0.035)]
                      ]) 
        
        # Obtain the BODY Jacobian
        Jb = Tbs_adj @ Js
        

        return Tbs, Jb

    def geom_IK(self, Td):
        """
        Gives joint angles using the geometric method.
        """ 
        # Get the end-effector coordinates
        Xt = Td[0,3]; Yt = Td[1,3]; Zt = Td[2,3]

        # Get the end-effector approach vector
        ax = Td[0,0]; ay = Td[1,0]; az = Td[2,0]

        # Get the wrist vector
        wx = Xt-self.L4*ax
        wy = Yt-self.L4*ay
        wz = Zt-self.L4*az
        
        # Calculate some intermediate variables
        r = np.sqrt((np.power(wx,2))+(np.power(wy,2)))
        h=wz-self.L1
        c = np.sqrt((np.power(r,2))+(np.power(h,2)))
        beta = np.arctan2(self.Lm,self.L2)
        psi = np.pi/2-beta
        Lr = np.sqrt((np.power(self.Lm,2))+(np.power(self.L2,2)))
        phi= np.arccos((np.power(c,2)-np.power(self.L3,2)-np.power(Lr,2))/(-2*Lr*self.L3))
        gamma = np.arctan2(h,r)
        alpha = np.arccos((np.power(self.L3,2)-np.power(Lr,2)-np.power(c,2))/(-2*Lr*c))
        theta_a=np.arctan2(np.sqrt(np.power(ax,2)+np.power(ay,2)),az)

        # Get corresponding joint angles using geometry (elbow-up solution)
        q1 =  np.arctan2(Yt,Xt)# Waist angle
        q2 =  (np.pi/2)-beta-alpha-gamma# Shoulder angle
        q3 =  np.pi-psi-phi# Elbow angle
        q4 =  theta_a-q2-q3-np.pi/2# Wrist angle

        # Return angles
        return [q1, q2, q3, q4]

    def num_IK(self, Tsd, InitGuess):
        """
        Gives joint angles using numerical method.
        """
        for i in range(1000):
            # Calculate the end-effector transform (Tsb) evaluated at the InitGuess using the helper functions that you wrote at the beginning.
            # Tsd = 
            Tbs, Jb = self.body_jacobian(InitGuess)

            # Compute the body twist
            matrix_Vb = logm(Tbs @ Tsd) 
            
            Vb = self.twist_vector_from_twist_matrix(matrix_Vb) # use the helper function at the beginning to extract the vector

            # Compute new angles
            NewGuess = InitGuess + np.linalg.pinv(Jb) @ Vb

            # Check if you're done and update initial guess
            if(np.linalg.norm(abs(NewGuess-InitGuess)) <= 0.001):
                print(f"Iteration number: {i} \n")
                return [NewGuess[0], NewGuess[1], NewGuess[2], NewGuess[3]] 
            else:
                InitGuess = NewGuess
        print(f"Iteration number: {i} \n")
        print('Numerical solution failed!!')
    
def main():
    # Determine the desired end-effector transform
    Td_grasp = np.array([[1, 0, 0, 0.1],
                            [0, 1, 0, 0.1],
                            [0, 0, 1, 0.1],
                            [0, 0, 0, 1]]) # Gripping location
    Td_release = np.array([[1, 0, 0, 0.2],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0.2],
                            [0, 0, 0, 1]]) # Throwing location
    


    ################################################################
    # This is material we need to edit
    Td_grasp2 = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    Td_release2 = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    Td_grasp3 = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    Td_release3 = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    Td_grasp4 = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    Td_release4 = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    ################################################################



    # Create experiment objects (use robot API + our custom API)
    bot = InterbotixManipulatorXS(
        robot_model='px100',
        group_name='arm',
        gripper_name='gripper'
    )
    my_api = ourAPI()

    # Start with home position
    bot.arm.go_to_home_pose()

    # toggle between the geometric method and the numerical method below
    # record the answers that you get in both cases. report your observations. 

    # Go to gripping position and grip
    # joint_positions = my_api.geom_IK(Td_grasp) # Geometric inverse kinematics
    print("\n\n\n\nGRASPING:")
    joint_positions = my_api.num_IK(Td_grasp, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.grasp(2.0) # Grip
    
    # Go to throwing position and throw
    # joint_positions = my_api.geom_IK(Td_release) # Geometric inverse kinematics
    print("\n\n\n\nRELEASE:")
    joint_positions = my_api.num_IK(Td_release, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.release(2.0) # Release
    


    ################################################################
    # This is material we need to edit
    bot.arm.go_to_home_pose()
    joint_positions = my_api.num_IK(Td_grasp2, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.grasp(2.0) # Grip
    joint_positions = my_api.num_IK(Td_release2, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.release(2.0) # Release

    bot.arm.go_to_home_pose()
    joint_positions = my_api.num_IK(Td_grasp3, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.grasp(2.0) # Grip
    joint_positions = my_api.num_IK(Td_release3, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.release(2.0) # Release

    bot.arm.go_to_home_pose()
    joint_positions = my_api.num_IK(Td_grasp4, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.grasp(2.0) # Grip
    joint_positions = my_api.num_IK(Td_release4, np.array([0.0, 0.0, 0.0, 0.0])) # Numeric inverse kinematics
    bot.arm.set_joint_positions(joint_positions) # Set positions
    bot.gripper.release(2.0) # Release
    ################################################################



    # End mission
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()
    bot.shutdown()


if __name__ == '__main__':
    main()