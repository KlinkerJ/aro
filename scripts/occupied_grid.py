import numpy as np
import random
from math import floor
from matplotlib import pyplot as plt
from matplotlib import colors

# generate grid
x = 20
y = 30
px = 1
py = 1
nx = int(x/px)
ny = int(y/py)
field_array = np.zeros((nx, ny))

# generate one random measurement
def generate_measurement(x=(0,20), y=(0,30), s=(0,3)):
    x = round(random.uniform(x[0], x[1]), 2)
    y = round(random.uniform(y[0], y[1]), 2)
    s = round(random.uniform(s[0], s[1]), 2)
    return [x,y,s]

# generate 10 measurements and append to corresponding field_array element
for i in range(10):
    mx, my, ms = generate_measurement()
    mx, my = floor(mx), floor(my)
    field_array[mx, my] = ms

# create occupied grid
cmap = colors.ListedColormap(['Green','Yellow'])
plt.figure(figsize=(6,6))
plt.pcolor(field_array[::-1],cmap=cmap,edgecolors='k', linewidths=1)
plt.xticks(np.arange(0,y,step=py))
plt.yticks(np.arange(0,x,step=px))
plt.show()

plt.imshow(field_array)
plt.show()