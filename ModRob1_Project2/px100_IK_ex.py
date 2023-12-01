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
        expS1 = screw_axis_to_transformation_matrix(S[0], angles[0])
        expS2 = screw_axis_to_transformation_matrix(S[1], angles[1])
        expS3 = screw_axis_to_transformation_matrix(S[2], angles[2])
        expS4 = screw_axis_to_transformation_matrix(S[3], angles[3])
        
        # Obtain Tsb and Tbs using the multiplication of the exponentials
        Tsb = expS1 @ expS2 @ expS3 @ expS4 @ M
        Tbs = np.linalg.inv(Tsb)

        # Obtain the adjoint of Tbs
        R_for_adj = Tbs[:3, :3]
        p_for_adj = Tbs[:3, 3]
        p_for_adj_bracket = [[0, -p_for_adj[2], p_for_adj[1]], 
                            [p_for_adj[2], 0, -p_for_adj[0]], 
                            [-p_for_adj[1], p_for_adj[0], 0]]
        zero_matrix = np.zeros(3)
        Tbs_adj = [[R_for_adj,                      zero_matrix], 
                   [p_for_adj_bracket @ R_for_adj, R_for_adj]]

        # Obtain the SPACE Jacobian
        Js = [[0, -np.sin(angles[0]), 0, 0],
              [0, np.cos(angles[0]), 1, 1],
              [1, 0, 0, 0],
              [0, -0.08945*np.cos(angles[0]), 0.035*np.sin(angles[1])-0.1*np.cos(angles[1]) - 0.08945, 0.1*np.sin(angles[2])-0.18945],
              [0, -0.08945*np.sin(angles[0]), 0, 0],
              [0, 0, 0.035*np.cos(angles[1])+0.1*np.sin(angles[1]), 0.1*np.cos(angles[2]+0.035)]
            ] 
        
        # Obtain the BODY Jacobian
        Jb = Tbs_adj @ Js
        

        return Jb