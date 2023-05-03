import numpy as np
from math import floor
'''
def opt2position(position, size):
    x = floor(position[0] / size)
    y = floor(position[1] / size)
    return (x, y)

def position2cell(vecPosition, size, cols):
    x = floor(vecPosition[0] / size)
    y = floor(vecPosition[1] / size)
    return cols * x + y

def decimal2int(value):
    return int(value)

def cart2state(positionGrid, numCols):
    return positionGrid[0] * numCols + positionGrid[1]


x = np.linspace(0, 8, 100)
y = np.linspace(0, 4, 100)

for i in range(len(x)):
    print(opt2position((x[i], y[i]), .2))
print('fim')
for i in range(len(x)):
    print(position2cell(opt2position((x[i], y[i]), .2), .2, 10))
'''

def opt2position(x_max, y_max, size, position):
    n_cols = int(x_max / size)
    n_rows = int(y_max / size)
    
    #print(n_cols, n_rows)
    current_10x = position[0] * 10
    current_10y = position[1] * 10
    size_10 = size * 10

    print(position)
    print(int(current_10x/size_10), int(current_10y/size_10))

    print((n_cols + 1) * int(current_10y/size_10) + int(current_10x/size_10))
    print(' ')

    return None

#x = np.linspace(0, 8, 81)
#y = np.linspace(0, 5, 51)
#for jj in y:
#    for ii in x:
#        opt2position(8,5, .2, (ii, jj))
#    print('  ')
#    if jj > .1:
#        break


a = np.array([1, 2, 3, 4])
b = np.array([4, 5, 6, 7])
c = set(a) & set(b)
print((np.where(a == 1)[0][0]))
print(c)

print([elemento for elemento in a if elemento not in b])
