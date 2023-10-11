# this is about creating a simple robot arm class

# here the RobotArm class has the attributes of name, length, weight, and color and also position
# we use the methods move and displaye_info to rotate the robot and display the information
# the __init__ method serves as the constructor and initializes the robot arm's attributes when an object is created from the class
# the move method takes an angle parameter and updates the robot arm's position by adding the angle to the current position
# the display_info method prints the robot arm's name, length, weight, color, and current position
# here suppost that our robot arm only has one link and one joint --> 1 degree of freedom
class RobotArm:
    def __init__(self, name, length, weight, color):
        self.name = name
        self.length = length
        self.weight = weight
        self.color = color
        self.position = 0

    def move(self, angle):
        self.position += angle

    def display_info(self):
        print(f"Name: {self.name}")
        print(f"Length: {self.length} cm")
        print(f"Weight: {self.weight} kg")
        print(f"Color: {self.color}")
        print(f"Position: {self.position} degrees")


"""
note that whatever I write between these three times quotation marks will not be executed
this is another way to write comments like using #
note that this is for the humans only to understand the code

"""

# now let's create an object of the RobotArm class and utilize its attributes and methods

# here we create an object arm1 of the RobotArm class 
# we provided it with arguments Armrob, 50, 10, and Black to the constructor 
arm1 = RobotArm("Armrob", 50, 10, "Black")
# here we call the move method to update the arm's position by 45 deg and display its information using the display_info method
arm1.move(45)
arm1.display_info()

# here we again call move method to further change the arm's position by -15 deg and display the updated information
arm1.move(-15)
arm1.display_info()