import numpy as np

# Set the numpy print preferences
np.set_printoptions(suppress=True)

# Define your rotation matrix
R = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]])

# Function
def w_hat_and_theta_given_R(R):
    # Define verbosity
    verbose = False

    # To obtain theta in RADIANS
    theta1 = np.arccos((R[0][0] + R[1][1] + R[2][2] - 1)/2)
    theta2 = -theta1 # Because it's cosine

    # Scenario 1: sin(theta) != 0
    tolerance = 1e-5
    if np.sin(theta1) <= (0-tolerance) or (np.sin(theta1) >= (0+tolerance)):
        
        w_hat_bracket_1 = 1/(2*np.sin(theta1)) * (R - R.T)
        w1_hat_1 = w_hat_bracket_1[2][1]
        w2_hat_1 = w_hat_bracket_1[0][2]
        w3_hat_1 = w_hat_bracket_1[1][0]
        w_hat1 = [w1_hat_1, w2_hat_1, w3_hat_1]
        
        w_hat_bracket_2 = 1/(2*np.sin(theta2)) * (R - R.T)
        w1_hat_2 = w_hat_bracket_2[2][1]
        w2_hat_2 = w_hat_bracket_2[0][2]
        w3_hat_2 = w_hat_bracket_2[1][0]
        w_hat2 = [w1_hat_2, w2_hat_2, w3_hat_2]

        # Print the result!
        if verbose:
            print(f"\n---------OPTION 1---------")
            print(f"\nWith theta1 = {theta1} rad, w_hat_bracket is = ")
            print(w_hat_bracket_1)
            print(f"And the w_hat is = {w_hat1}\n")
            print(f"\n---------OPTION 2---------")
            print(f"\nWith theta2 = {theta2} rad, w_hat_bracket is = ")
            print(w_hat_bracket_2, "\n")
            print(f"And the w_hat is = {w_hat2}\n")

        # Return only the first option for simplicity
        theta = theta1
        w_hat = w_hat1
        w_hat_bracket = w_hat_bracket_1
        return theta, w_hat, w_hat_bracket

    # Scenario 2: theta = k*pi AND k is an EVEN integer
    elif theta1/np.pi % 2 == 0:
        print("\nRESULT: w_hat cannot be obtained because R = I and the axis of rotation is undefined!\n")

    # Scenario 3: theta = k*pi AND k is an ODD integer
    elif theta1/np.pi % 2 != 0:
        # We need to find the individual components of [w_hat]
        w1_hat_pos1 = np.sqrt((R[0][0] + 1)/2)
        w1_hat_pos2 = -np.sqrt((R[0][0] + 1)/2)
        w2_hat_pos1 = np.sqrt((R[1][1] + 1)/2)
        w2_hat_pos2 = -np.sqrt((R[1][1] + 1)/2)
        w3_hat_pos1 = np.sqrt((R[2][2] + 1)/2)
        w3_hat_pos2 = -np.sqrt((R[2][2] + 1)/2)

        # Go through all the potential combinations of (w1_hat, w2_hat, w3_hat)
        potential_combinations = [[w1_hat_pos1, w2_hat_pos1, w3_hat_pos1],
                                [w1_hat_pos1, w2_hat_pos1, w3_hat_pos2],
                                [w1_hat_pos1, w2_hat_pos2, w3_hat_pos1],
                                [w1_hat_pos1, w2_hat_pos2, w3_hat_pos2],
                                [w1_hat_pos2, w2_hat_pos1, w3_hat_pos1],
                                [w1_hat_pos2, w2_hat_pos1, w3_hat_pos2],
                                [w1_hat_pos2, w2_hat_pos2, w3_hat_pos1],
                                [w1_hat_pos2, w2_hat_pos2, w3_hat_pos2]]

        # Check which are actual solutions
        solutions = []
        for combination in potential_combinations:
            w1_hat = combination[0]
            w2_hat = combination[1]
            w3_hat = combination[2]
            w_hat = np.array([[2*np.power(w1_hat, 2)-1, 2*w1_hat*w2_hat, 2*w1_hat*w3_hat],
                            [2*w1_hat*w2_hat, 2*np.power(w2_hat, 2)-1, 2*w2_hat*w3_hat], 
                            [2*w1_hat*w3_hat, 2*w2_hat*w3_hat, 2*np.power(w3_hat, 2)-1]])
            
            # Check if this is equal to R, and if so, check if it has been added to solutions. If NOT, then add it as a potential solution:
            if np.allclose(R, w_hat, atol=tolerance):
                if [w1_hat, w2_hat, w3_hat] not in solutions:
                    solutions.append([w1_hat, w2_hat, w3_hat])
            
            # Stop if two solutions have been found
            if len(solutions) >= 2:
                break
            
        # Save them with the appropriate name
        w_hat1 = solutions[0]
        w_hat2 = solutions[1]

        # Print the result!
        if verbose:
            print(f"\nWith theta1 = {theta1} rad, w_hat is = ")
            print(w_hat1)
            print(f"\nWith theta2 = {theta1} rad, w_hat is = ")
            print(w_hat2, "\n")

        # Return only the first option for simplicity
        theta = theta1
        w_hat = w_hat1
        w_hat_bracket = np.array([[0, -w_hat[2], w_hat[1], w_hat[2], 0, -w_hat[0], -w_hat[1], w_hat[0], 0]])
        return theta, w_hat, w_hat_bracket
    
# Test
theta, w_hat, w_hat_bracket = w_hat_and_theta_given_R(R)
print(w_hat_bracket)