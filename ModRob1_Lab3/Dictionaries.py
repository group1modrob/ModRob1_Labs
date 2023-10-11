# a dictionary that maps robot arm joints to their corresponding min, max, and default joint angles
import math as m

arm_dict = {
    "joint1": {
        "min" : -m.pi/2,
        "max" : m.pi/2,
        "default" : 0,
    },

        "joint2": {
        "min" : -m.pi,
        "max" : m.pi,
        "default" : 0,
    },

        "joint3": {
        "min" : -m.pi/2,
        "max" : m.pi/2,
        "default" : 0,
    },

}


# now we can access the keys and values in this dictionary
joint_angles_1 = arm_dict["joint1"]
print(joint_angles_1)

# you can also access to individual angles within each joint in the dictionary
# this gives the max angle of joint 2
print(arm_dict["joint2"]["max"])

# print the min angle of joint3
print(arm_dict["joint3"]["min"])