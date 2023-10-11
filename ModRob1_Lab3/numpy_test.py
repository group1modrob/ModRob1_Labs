# now let's test working with numpy

import numpy as np

v1 = np.array([1, 2, 3, 4, 5, 6])
print(v1)

# this way you can see that the v is saved as an array istead of a list 

# let's define another vector
v2 = np.array([7, 8, 9, 10, 11, 12])

# we can do several vector operations
print(v2)
print(v1 + v2)

# here the vectors should be of the same length

# element-wise, matrix multiplications
print (v1 * v2)

print(v1.shape) # it shows that v1 is 6 times nothing matrix 
# if we want to make it a 6 by 1 vector:
v1 = np.array([[1], [2], [3], [4], [5], [6]])
print(v1.shape)

# we do the same with v2
v2 = np.array([[7], [8], [9], [10], [11], [12]])
print(v2.shape)

# in order to do matrix multiplication, we should multiply one by the transpose of the other 

print(np.matmul(v1,v2.T))
print("or")
print(np.matmul(v1,np.transpose(v2))) # this matrix is 6 by 6
print(v2.T.shape) # see that it is a 1 by 6 matrix 

# @ also means matrix multiplication 
print(v1.T @ v2) # which is a 1 by 1 vector or just a number

# if we want to write a for example 2 by 6 matrix:

x = np.array([[[1], [2], [3], [4], [5], [6]],[[7], [8], [9], [10], [11], [12]]])
print(x.shape)