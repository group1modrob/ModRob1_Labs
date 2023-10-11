# this is for the for loops section
# these are very basic and easy examples
# more to come in the classes and objects section

# we can use for loop with strings
# a loop that iterates over each character in the string "Robotics" and prints each character on a separate line

for letter in "Robotics":
    print(letter)

# we can also use for loops with other data types like lists
# Here we defined a list called robotics_engineers containing the names of several robotics engineers, and then we used a for loop to # iterate through each engineer's name in the list and print each name on a separate line. 

robotics_engineers = ["Tim", "Hannah", "John", "Bella"]
for robotic_engineer in robotics_engineers:
    print(robotic_engineer)

# let's do another for loop with the indexes
# Here, we are using a for loop to iterate through the indices of the robotics_engineers list, and for each index, we are printing both the # index itself and the corresponding element (name of the robotic engineer) from the list.

for index in range(len(robotics_engineers)):
    print(index)
    print(robotics_engineers[index])

# you can loop through a series of numbers or range of numbers
# The range function in Python generates a sequence of numbers starting from the first argument and ending at one less than the second argument. So the numbers generated will be from 3 to 9.
for number in range(3, 10):
    print(number)