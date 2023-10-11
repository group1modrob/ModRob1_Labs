import math as m

# this function will get the radius of the wheel and returns the area of the wheel
def calc_wheel_area(radius):
    area = m.pi * radius**2
    return area

# now we want to call this function
wheel_radius = 0.2 # 0.2 meters
wheel_area = calc_wheel_area(wheel_radius)
print("The area of the wheel is", wheel_area)