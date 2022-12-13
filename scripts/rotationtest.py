import numpy as np

x = 0
y = 1

theta = np.deg2rad(45)
rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) # Drehmatrix
new_x = (rot[0][0] * x) + (rot[0][1] * y)
new_y = (rot[1][0] * x) + (rot[1][1] * y)
print(new_x, new_y)