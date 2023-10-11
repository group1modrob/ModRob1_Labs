# this is about inheritance in Python

# suppose we want to create a program to simulate different types of robots
# first we will create a base class called Robot that defines common characteristics or robots

class Robot:
    def __init__ (self, name, DOFs):
        self.name = name 
        self.DOFs = DOFs

    def introduce(self):
        print(f"I am {self.name}, a {self.DOFs} DOF robot.")

    
    def move(self):
        print("I can move.")