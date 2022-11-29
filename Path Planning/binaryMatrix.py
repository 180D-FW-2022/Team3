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
        possible=[(x,y) for x,y in [(i+1,j),(i-1,j),(i,j+1),(i,j-1)] if 0<=x<m and 0<=y<n and grid[x][y]==0]
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
alley_h = np.zeros((1*2, 7*2))
table = 1*np.ones((1*2, 1*2))
table_alley = 2*np.ones((1*2, 1*2))

alleyAndTable_f = np.concatenate((table_alley, table, table_alley, table, table_alley, table, table_alley, table, table_alley), axis=0)
    
alleyAndV = np.concatenate((alley_v, alleyAndTable_f, alley_v, alleyAndTable_f, alley_v, alleyAndTable_f, alley_v), axis=1)

alleyF = np.concatenate((alley_h,alleyAndV,alley_h), axis = 0)

print(alleyF)



plt.ion()
fig, ax = plt.subplots()
x, y = [],[]
sc = ax.scatter(x,y)
colorsList = ['w', 'r', 'g']
cmap = mpl.colors.ListedColormap(colorsList)
plt.imshow(alleyF, cmap=cmap)
plt.xlim(-2,20)
plt.ylim(0,25)

plt.draw()
for i in range(2,20):
    arr = helper(alleyF, i, 0, 14, 5)
    x.clear()
    y.clear()
    for i in arr: 
            y.append(i[0])
    for i in arr: 
            x.append(i[1])
    sc.set_offsets(np.c_[x,y])
    fig.canvas.draw_idle()
    #print(arr)
    plt.pause(1)

plt.waitforbuttonpress()