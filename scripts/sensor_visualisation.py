import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import db
from peewee import *
from itertools import groupby
import sektoren

# read the height data from the SQL-database
database = SqliteDatabase('/home/lennart/catkin_ws/src/aro/db/segments.sqlite')
segments = db.Segments.select().execute()

# sort the data in the SQL-database: 
# first: sort by sm_y, second: group by sm_y, third: sort by sm_x
keyfunc = lambda x:x.sm_y
keyfunc2 = lambda x:x.sm_y
keyfunc3 = lambda x:x.sm_x
mylist = sorted(segments, key=keyfunc)
grouper = groupby(mylist, keyfunc2)

# write the sorted data from the database into an array of type arr = [[],[],...]
result = []
for a, files in grouper:
    r = sorted(list(files), key=keyfunc3)
    z = []
    id = []
    for i in r:
        z.append(i.height)
    result.append(z)

# some preperation for the grid in the visualisation
# and the axes labels
x_label = []
x_value = 1.0
for i in result[[0][0]]:
    x_label.append(x_value)
    x_value = x_value+1.0

y_label = []
y_value = 1.0
for i in result:
    y_label.append(y_value)
    y_value = y_value+1.0

# creat plot and set axes labels
constants = db.get_constants()
fig, ax = plt.subplots()
im = ax.imshow(result[::-1], cmap='YlGn')
ax.set_xticks(np.arange(len(x_label)))
ax.set_xticklabels(labels=(np.arange(constants['min_x']-constants['segmentsize']/2+0.5, constants['max_x']+constants['segmentsize']/2+0.5)))
ax.set_yticks(np.arange(len(y_label)))
ax.set_yticklabels(labels=(np.arange(constants['min_y']-constants['segmentsize']/2+0.5, constants['max_y']+constants['segmentsize']/2+0.5)))
ax.set_ylabel('field height:'+' '+ str(constants['max_y']-constants['min_y']+constants['segmentsize']) +' '+'m', fontsize=14, labelpad=14)
ax.set_xlabel('field width:'+' '+ str(constants['max_x']-constants['min_x']+constants['segmentsize']) +' '+'m', fontsize=14, labelpad=14)
#ax.invert_yaxis()

# genereate grid based on the fild size and colorize the generated grid
grid_x_value = 0.5
grid_y_value = 0.5

for i in x_label:
    ax.axvline(grid_x_value, color='k', linewidth=1)
    grid_x_value = grid_x_value+1

for i in y_label:
    ax.axhline(grid_y_value, color='k', linewidth=1)
    grid_y_value = grid_y_value+1

# write the array values into the plot
for (i, j), z in np.ndenumerate(result[::-1]):
    ax.text(j, i, '{:0.2f}'.format(z), ha='center', va='center')

# set plot title and show the plot
ax.set_title('Height map of the current field', fontsize=16)
cbar = ax.figure.colorbar(im)
cbar.set_label('growth height'+' '+'[m]', rotation = 270, labelpad=26, fontsize=12)
fig.tight_layout()
plt.show()