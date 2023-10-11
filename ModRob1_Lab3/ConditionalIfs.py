# in this example we want to write a function that gives us the minimum number of three numbers that we pass to it

# first we define a function called "min_num" 
def min_num(a,b,c):
    #there are 4 conditions for this function
    #the first condition is about a as the smallest number. If it match, it will return "The smallest number is a"
    if a < b and a < c:
        return "The smallest number is " + str(a)
    # This conidtion is for a as the smallest number. If the conidtion match, it will return "The smallest number is a"
    # if the function doesn't match this coniditon, then it will try other following coniditons
    elif b < a and b < c:
    # This conidtion is for b as the smallest number. If the conidtion match, it will return "The smallest number is b"
    # If the function doesn't match this coniditon, then it will try next conidiotn that c is the smallest number
        return "The smallest number is " + str(b)
    # This conidtion is for b as the smallest number. If the conidtion match, it will return "The smallest number is c"
    # If the function doesn't match this coniditon, then it will try other following coniditons
    elif c < a and c < b:
        return "The smallest number is " + str(c)
    # If the function doesn't match all the previous conditions, it will return "They are equal"
    else:
        return "They are equal"
    

# then we can try how the function works with all the conidiotions we set
print(min_num(-2,-1,0))
print(min_num(0,-4,1))
print(min_num(-1,-2,-6))
print(min_num(-0.5,-0.5,-0.5))