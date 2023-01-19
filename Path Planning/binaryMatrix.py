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

alleyF = np.zeros((20,20))

#print(alleyF)

robotDir = 0 #90, 180, 270.
needMoveDir = 0

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
arr = helper(alleyF, 14, 5, 14, 4) #y-start, x-start, y-end, x-end
cnt = 0
arr_direction_y = []
arr_direction_x = []
arr_angle = []
for i in arr:
    if(cnt>0):
        y_dir = i[0]-arr[cnt-1][0]
        x_dir = i[1]-arr[cnt-1][1]
        arr_direction_y.append(y_dir)
        arr_direction_x.append(x_dir)
        angle = 0
        if(y_dir == 1):
            angle = 0
        elif(y_dir == -1):
            angle = 180
        elif(x_dir == 1):
            angle = 90
        elif(x_dir == -1):
            angle = 270
        arr_angle.append(angle)
    cnt+=1

j_prev = arr_angle[0]
distance = 0
arr_distance = []
arr_angle_clean = []
counter = 0
for j in arr_angle:
    counter+= 1
    if(j == j_prev and counter <= len(arr_angle)):
        j_prev = j
        distance+=0.5   
        if(counter == len(arr_angle)):
            arr_distance.append(distance)
            arr_angle_clean.append(j_prev)
            distance = 0
    else:    
        if(distance != 0):
            arr_distance.append(distance)
            arr_angle_clean.append(j_prev)
            distance = 0
        j_prev = j
print(arr_distance)
print(arr_angle_clean)

x.clear()
y.clear()
for i in arr: 
        y.append(i[0])
for i in arr: 
        x.append(i[1])
sc.set_offsets(np.c_[x,y])
fig.canvas.draw_idle()
#print(arr)

plt.waitforbuttonpress()