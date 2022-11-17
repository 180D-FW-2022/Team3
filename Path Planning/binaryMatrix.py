import collections

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np

def helper(grid, start_m, start_n, goal_m, goal_n):
    m,n=len(grid),len(grid[0])
    deque=collections.deque([[(start_m,start_n)]])
    seen=set()
    while deque:
        arr=deque.popleft()
        i,j=arr[-1]
        if (i,j)==(goal_m,goal_n):
            return arr
        seen.add((i,j))
        possible=[(x,y) for x,y in [(i+1,j),(i-1,j),(i,j+1),(i,j-1)] if 0<=x<m and 0<=y<n and grid[x][y]!=1]
        for x,y in possible:
            if (x,y) not in seen:
                deque.append(arr+[(x,y)])
        
grid=[
    [1,0,0,1,0,0,1,1],
    [1,0,0,1,0,0,0,0],
    [1,0,0,1,0,0,1,1],
    [1,0,0,1,0,1,1,1],
    [1,0,0,0,0,1,1,1]
]
alley_v = np.zeros((9*2,1*2))
alley_h = np.zeros((1*2, 10*2))
table = np.ones((1*2, 1*2))
table_alley = np.zeros((1*2, 1*2))

alleyAndTable_f = np.concatenate((table_alley, table, table_alley, table, table_alley, table, table_alley, table, table_alley), axis=0)
    
alleyAndV = np.concatenate((alley_v, alleyAndTable_f, alley_v, alleyAndTable_f, alley_v), axis=1)



arr = helper(alleyAndV, 0, 0, 14, 5)

x = []
y = []
for i in arr: 
        y.append(i[0])
for i in arr: 
        x.append(i[1])


plt.scatter(x, y)

# random data
x = alleyAndV

# fig, ax = plt.subplots()

# define the colors
cmap = mpl.colors.ListedColormap(['w', 'b'])

# create a normalize object the describes the limits of
# each color
bounds = [0., 0.5, 1.]
norm = mpl.colors.BoundaryNorm(bounds, cmap.N)

# plot it
plt.imshow(x, interpolation='none', cmap=cmap, norm=norm)
plt.show()