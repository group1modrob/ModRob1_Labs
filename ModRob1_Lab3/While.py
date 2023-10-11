# in this example we want to simulate a situation where a robot moves forward until it encounters an obstacle
import time
import random 

# the move_robot() function simulates the robot's movement

def move_robot(distance):
    print("Moving robot forward ...")
    # now we want to simulate the time it takes for the robot to move forward by giving some delay
    time.sleep(1)
    print(f"The robot has moved {distance} units.")
    # note that f"..." is used to create formatted string literals aka f-strings 
    # it is introduced in python 3.6 and allows you to embed expressions inside string literals


# now we should write a function for obstacle detection
def detect_obstacle():
    # simulating obstacle detection
    obstacle = random.choice([True, False])
    return obstacle 
# the random.choice() function is part of the random module in python
# it allows you to select a random element from a sequence such as a list, tuple or string 
# because we are simulating a situation here and we do not have real obstacles we randomly choose whenever an obstacle is present

# robot's initial position
position = 0

# we will keep moving the robot until an obstacle is detected
while not detect_obstacle():
    # now we should generate a random distance to move fw (because we are simulating and it's not a real scenario)
    distance = random.randint(1, 5)
    # here we call the move_robot() function to move the robot fw by the randomly generated distance 
    move_robot(distance)
    # now we will add this distance to the position of the robot
    # here we update the position of the robot
    position += distance

# here the while loop continues to execute as long as the detect_obstacle() function returns False meaning that there is no obstacle. 
# if it becomes True, then it means that there is an obstacle and the loop is exited and a message will print showing that there is an obstacle
# and the control system should take action accordingly and commands the actuators to stop the robot


print("Obstacle is detected! Stopping the robot.")
# we also print the final position of the robot
print(f"The final position of the robot is {position}")