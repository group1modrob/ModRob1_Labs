# First we will create two different types of numbers: integers (ints) and floats. Ints have no decimal spaces, floats can have up to 7 decimal digits
int_example = 100
float_example = 0.2

# When printing things to the console, we technically cannot put Strings and Numbers in the same print statement
# The numbers and any other variable type need to be changed to String before being used in the statement
# You do that with the str() command and whatever is given as an argument will be changed to a string
print("My first example was an integer with a value of " + str(int_example) + ".\n The second example is a float with the value of " + str(float_example) + ".") 

# Another type of variable are String variables. Here I will create an example:
example_string = "Here is an example"
# And now I can use it directly in a print statement because it is already a String
print("My string example will contain the word example because I am a redundant person: " + factory_name + ".")

# A list is a variable type that can contain many values OR variables in it. For example, we can create a list of Strings:
example_list = ["Here", "is", "an", "example", "list"]

# We can then print this list
print(example_list)

# We can access individual elements of the list using indexing with []. For example, printing the first element in the list (index starts at 0)
print("The first string in my list is: " + example_list[0])

# We can also use indexing to print all the elements UP TO index X. If I want to print the first 4 elements (index 0 - 3)
# print the first three items in the list
print(example_list[:4]) # Not inclusive of the index listed here

# We can append elements to the list as needed using the .append command
example_list.append("and it could be the final one.")
print(example_list)

# We can remove the big element we just added by using the remove command
robot_brands.remove("and it could be the final one.")
print(example_list)

# We can insert elements into a specific index in the list
example_list.insert(4, "of a")
print(example_list)

# We can sort in place all the elements in ascending order using the sort() function
example_list.sort()
print(example_list) # And now the sentence will not make any sense

# We can reverse the order of the elements in the list
example_list.reverse()
print(example_list) # The sentence will still make no sense. Organizing it properly would require some code writing! 

# If we want to know the length of the list (how many elements the list has)
print(len(example_list))

# And we can also append lists to the list we already have

# we now want to append another list to the robot_brands list
robot_brands_1 = ["Yaskawa", "Staubli"]
robot_brands.extend(robot_brands_1)
print(robot_brands)

# we want to get rid of the last element of the robot_brands list
robot_brands.pop()
print(robot_brands)

# find the index of the value "ABB"
print(robot_brands.index("ABB"))

# now let's find out how many times "ABB" is repeated 
print(robot_brands.count("ABB"))

# we want to make a copy of the robot_brands list
robot_brands2 = robot_brands.copy()
print(robot_brands)
print(robot_brands2)


# we can iterate through a list with a for loop
for robot_brand in robot_brands:
    print(robot_brand)


# we can remove everything in a list with clear function
robot_brands.clear()
print(robot_brands)