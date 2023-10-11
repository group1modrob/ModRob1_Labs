from math import * 
import math as m

# Defining Values
num_int = 10 # this is an integer
num_float = 0.5 # this is a float

# Basic Arithmitic (+ = addition; - = subtraction; / = division; * = multiplication; ** = raised to the power of)
print(num_int * (num_int + num_float))

# % is the modulus operator --> gives the remainder of a division
print(num_int % 3)

# max and min --> larger or smaller of the two numbers
print(max(num_int,9))

# abs --> absolute value
print(abs(-num_int))

# pow gets the 1st num to the power of the second num
print(pow(num_int,2))

# round --> rounds a number down or up
print(round(num_float))

# floor will chop off the decimal point
print(floor(num_float))

# ceil will round the number up
print(ceil(num_float))

# sqrt --> square root
print(m.sqrt(num_int))

# combining numbers with strings
print("There are " + str(num_int) + " arm robots in the factory, "+ str(num_float) + " of them are broken.") 

# or you can write this like the following
print("There are",num_int,"arm robots in the factory,",num_float,"of them are broken.") 

# using string variables
factory_name = "ABC"
print("The factory's name is " + factory_name)
# accessing different characters in a string
print("The first character in the factory name is " + factory_name[0])
print("The third character in the factory name is " + factory_name[2])

# using different functions with strings
print(factory_name.lower()) # this will print the factory_name in all lower case
print(factory_name.islower()) # checks to see if the factory name is all lower case

# we can combine different functions
print(factory_name.upper().isupper()) # first makes the factory name all upper and then checks if it is all upper

print(len(factory_name)) # prints out the length of the factory_name string

print(factory_name[0:len(factory_name)-1]) # prints the characters of the factory_name from the first character up to the one to the last character

print(factory_name.index("A")) # prints the index of the character A 

print(factory_name.replace("ABC", "DEF")) # replaces the name of the factory with DEF
