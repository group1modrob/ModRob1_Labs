# this is for the nested loop section
# nested loops --> loops inside the loops

# we can iterate over the elements of a 2D list using nested loops
# the outer loop iterates over the rows 
# the inner loop iterates over the columns 

# let's go back to the example of the 2D list
matrix = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]

for row in matrix:
    for element in row:
        print(element)