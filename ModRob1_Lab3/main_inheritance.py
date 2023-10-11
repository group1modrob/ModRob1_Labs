# let's now create some instances of these classes and see how they behave

from WheeledRobot import WheeledRobot
from FlyingRobot import FlyingRobot 

robot1 = WheeledRobot("Rover", 3, 4)
robot1.introduce() # this is the introduce function that we created in the Robot class and WheeledRobot class inherited from it
robot1.move() # this will provide its own definition of the move function and also the inherited move function from the Robot class


robot2 = FlyingRobot("DroneX", 6, 100)
robot2.introduce()
robot2.move()

# here you saw that the instance of WheeledRobot and FlyingRobot inherit the introduce and move methods from the Robot base class.
# they also have their own specialized implementations of the move method specific to them

# here you saw that by using inheritance we can easily extend and customize classes to represent different types of robots 
# while reusing common code from the base class