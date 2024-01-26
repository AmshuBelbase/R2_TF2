import numpy as np

# Define a 4x3 matrix
matrix_4x3 = np.array([[1, 2, 3],
                      [4, 5, 6],
                      [7, 8, 9],
                      [10, 11, 12]])

# Define a 3x1 matrix
matrix_3x1 = np.array([[2],
                      [3],
                      [4]])

# Perform matrix multiplication
result_matrix = np.dot(matrix_4x3, matrix_3x1)

# Display the result
print("Wheel 1:") # Front Right
print(result_matrix[0,0])
print("Wheel 2:") # Front Left
print(result_matrix[1,0])
print("Wheel 3:") # Back Left
print(result_matrix[2,0])
print("Wheel 4:") # Back Right
print(result_matrix[3,0])
