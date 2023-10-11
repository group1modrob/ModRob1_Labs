# this section is about tuples

robotic_engineers = ("Hannah", "Jim", "Bella", "John")
print(robotic_engineers)

# print the 2nd value in the tuple
print(robotic_engineers[1])

# unpack the tuple into the variables e1, e2, e3, and e4 that stand for engineers 1 through 4

e1, e2, e3, e4 = robotic_engineers
print(e1, e2, e3, e4)
print(e3)

# we cannot change anything inside the tuple
# robotic_engineers[1] = "Bob"

# we can make a list of tuples
robotic_partners = [(robotic_engineers[0],robotic_engineers[3]), (robotic_engineers[1], robotic_engineers[2])]
print(robotic_partners)
print(robotic_partners[0])