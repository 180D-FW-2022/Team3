from enum import Enum
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.heuristic import null

import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


class Move_type(Enum):
    NONE = 0
    ANGLE = 1
    DISTANCE = 2
    DIAGONAL = 3
    COMPLETE = 4

class Map:
    def __init__(self, gridArr, size = 20):
        self.gridArr = gridArr.copy()
        self.gridArrBinary = gridArr.copy()
        self.gridArrPathPlan = 0
        self.gridSize = 20
        self.table_dict = dict()
        self.table_total = 0
        self.table_count = 0
        self.home_coords = np.array([0,0]).astype(int)
        self.x = 0
        self.y = 0

        self.current_move_arr = []
        self.current_move_index = 0

        self._initMatrixs()

    def _initMatrixs(self):
        self._findTablesAndHome()
        self._createBinaryFromMatrix()
        self._createPathFindMatirx()
        
    def _findTablesAndHome(self):
        matrix_size = self.gridSize
        for row in range(matrix_size):
            for column in range(matrix_size):
                if(self.gridArr[row][column]>=10):
                    table_num = self.gridArr[row][column] - 10
                    if (table_num) not in self.table_dict:
                        self.table_dict[table_num] = [column, row]
                        self.table_total+=1
                        print(f"[  map  ] Table {table_num} found {self.table_dict[table_num]}")
                if(self.gridArr[row][column] == 4):
                    self.home_coords[0] = column
                    self.home_coords[1] = row
                    print(f"[  map  ] Home found: x: {self.home_coords[0]}, y: {self.home_coords[1]}")

    def _createBinaryFromMatrix(self):
        self.gridArrBinary[self.gridArrBinary > 9] = -1
        self.gridArrBinary[self.gridArrBinary == 4] = -1
        self.gridArrBinary[self.gridArrBinary > 0] = -2
        self.gridArrBinary[self.gridArrBinary == 0] = 1
        self.gridArrBinary[self.gridArrBinary == -1] = 1
        self.gridArrBinary[self.gridArrBinary == -2] = 0

    def _getHomePositionIndex(self):
        return self.home_coords
    
    def getHomePosition(self):
        return (self.home_coords[0] * 0.5, self.home_coords[1] * 0.5)
    

    def _getTablePositionIndex(self, tableNum):
        if tableNum in self.table_dict:
            return self.table_dict[tableNum]
        else:
            return [-1, -1]
        
    def getTablePosition(self, tableNum):
        location = self._getTablePositionIndex(tableNum)
        if(location[0] == -1):
            return (-1,-1)
        return (location[0] * 0.5, location[1] * 0.5)
    
    def setPosition(self, x, y):
        self.x = self.distanceToIndex(x)
        self.y = self.distanceToIndex(y)

    def setTarget(self, x, y): #motion arr calcualted with this.
        x_ind = self.distanceToIndex(x)
        y_ind = self.distanceToIndex(y)
        path = self._calcPath(self.x, self.y, x_ind, y_ind) #y-start, x-start, y-end, x-end
        if(path == None):
            return -1
        return 1
        
    def _calcLeastAngle(self, angle):
        to_ret = angle
        if(angle > 360):
            to_ret = angle%360
        if(angle < -360):
            to_ret = -1*(abs(angle)%360)
        if(angle == 270):
            to_ret = -90
        if(angle == -270):
            to_ret = 90
        return int(to_ret)

    def getNextMove(self):
        if(len(self.current_move_arr)>self.current_move_index):
            to_ret = self.current_move_arr[self.current_move_index]
            mov_type = Move_type.NONE
            if(self.current_move_index%2 == 0):
                mov_type = Move_type.ANGLE
                to_ret = self._calcLeastAngle(to_ret)
            else:
                mov_type = Move_type.DISTANCE
            self.current_move_index+=1
            return mov_type, to_ret
        else:
            return Move_type.COMPLETE, 0
        
    def getNextMove(self, currentRotation):
        if(len(self.current_move_arr)>self.current_move_index):
            to_ret = self.current_move_arr[self.current_move_index]
            mov_type = Move_type.NONE
            if(self.current_move_index%2 == 0):
                mov_type = Move_type.ANGLE
                to_ret -= currentRotation
                to_ret = self._calcLeastAngle(to_ret)
            else:
                mov_type = Move_type.DISTANCE
            self.current_move_index+=1
            return mov_type, to_ret
        else:
            return Move_type.COMPLETE, 0

    def distanceToIndex(self, dist):
        return int(dist*2)

    def _createPathFindMatirx(self, marginWeight = 4):
        grid_raw = self.gridArrBinary
        grid_raw_margined = grid_raw.copy()
        matrix_size = self.gridSize
        to_set_const = marginWeight
        for row in range(matrix_size): #populate margins of 0.5m or 1 block around items for path planning. 
            for column in range(matrix_size):
                current = grid_raw[row][column]
            # left = grid_raw[row][column-1]
            # right = grid_raw[row][column+1]
            # up = grid_raw[row-1][column]
            # down = grid_raw[row+1][column]
                if(row > 0 and row<(matrix_size-1) and column > 0 and column<(matrix_size-1)):
                    if(current == 1):
                        left = grid_raw[row][column-1]
                        right = grid_raw[row][column+1]
                        up = grid_raw[row-1][column]
                        down = grid_raw[row+1][column]
                        d1 = grid_raw[row-1][column-1] #up left diagonal
                        d2 = grid_raw[row-1][column+1] #up right diag
                        d3 = grid_raw[row+1][column-1] #down left diag
                        d4 = grid_raw[row+1][column+1] #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row == 0 and column > 0 and column<(matrix_size-1)): #top 
                    if(current == 1):
                        left = grid_raw[row][column-1]
                        right = grid_raw[row][column+1]
                        up = 1
                        down = grid_raw[row+1][column]
                        d1 = 1 #up left diagonal
                        d2 = 1 #up right diag
                        d3 = grid_raw[row+1][column-1] #down left diag
                        d4 = grid_raw[row+1][column+1] #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row == (matrix_size-1) and column > 0 and column<(matrix_size-1)): #bottom
                    if(current == 1):
                        left = grid_raw[row][column-1]
                        right = grid_raw[row][column+1]
                        up = grid_raw[row-1][column]
                        down = 1
                        d1 = grid_raw[row-1][column-1] #up left diagonal
                        d2 = grid_raw[row-1][column+1] #up right diag
                        d3 = 1 #down left diag
                        d4 = 1 #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row > 0 and row<(matrix_size-1) and column == 0): #left
                    if(current == 1):
                        left = 1
                        right = grid_raw[row][column+1]
                        up = grid_raw[row-1][column]
                        down = grid_raw[row+1][column]
                        d1 = 1 #up left diagonal
                        d2 = grid_raw[row-1][column+1] #up right diag
                        d3 = 1 #down left diag
                        d4 = grid_raw[row+1][column+1] #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row > 0 and row<(matrix_size-1) and column==(matrix_size-1)): #right
                    if(current == 1):
                        left = grid_raw[row][column-1]
                        right = 1
                        up = grid_raw[row-1][column]
                        down = grid_raw[row+1][column]
                        d1 = grid_raw[row-1][column-1] #up left diagonal
                        d2 = 1 #up right diag
                        d3 = grid_raw[row+1][column-1] #down left diag
                        d4 = 1 #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row == 0 and column == 0): #top left
                    if(current == 1):
                        left = 1
                        right = grid_raw[row][column+1]
                        up = 1
                        down = grid_raw[row+1][column]
                        d1 = 1 #up left diagonal
                        d2 = 1 #up right diag
                        d3 = 1 #down left diag
                        d4 = grid_raw[row+1][column+1] #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row == (matrix_size-1) and column == 0): #bottom left
                    if(current == 1):
                        left = 1
                        right = grid_raw[row][column+1]
                        up = grid_raw[row-1][column]
                        down = 1
                        d1 = 1 #up left diagonal
                        d2 = grid_raw[row-1][column+1] #up right diag
                        d3 = 1 #down left diag
                        d4 = 1 #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row == (matrix_size-1) and column == (matrix_size-1)): #bottom right
                    if(current == 1):
                        left = grid_raw[row][column-1]
                        right = 1
                        up = grid_raw[row-1][column]
                        down = 1
                        d1 = grid_raw[row-1][column-1] #up left diagonal
                        d2 = 1 #up right diag
                        d3 = 1 #down left diag
                        d4 = 1 #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
                if(row == 0 and column == (matrix_size-1)): #top right
                    if(current == 1):
                        left = grid_raw[row][column-1]
                        right = 1
                        up = 1
                        down = grid_raw[row+1][column]
                        d1 = 1 #up left diagonal
                        d2 = 1 #up right diag
                        d3 = grid_raw[row+1][column-1] #down left diag
                        d4 = 1 #down right diag
                        if(left == 0 or right == 0  or up == 0 or down == 0 or d1 == 0 or d2 == 0 or d3 == 0 or d4 == 0):
                            grid_raw_margined[row][column] = to_set_const
        self.gridArrPathPlan = grid_raw_margined.copy()
     
    def _calcPath(self,start_x_ind, start_y_ind, goal_x_ind, goal_y_ind):
        grid_ArrPathPlan = Grid(matrix=self.gridArrPathPlan.copy())
        start = grid_ArrPathPlan.node(start_x_ind, start_y_ind)
        end = grid_ArrPathPlan.node(goal_x_ind, goal_y_ind)
        finder = AStarFinder(diagonal_movement=DiagonalMovement.never,  heuristic=null)
        path, runs = finder.find_path(start, end, grid_ArrPathPlan)
        if(len(path) == 0):
            return None
        self._setMotionArray(path)
        return path
    
    def _setMotionArray(self, path):
        path_dist, path_ang = self._calcMoves(path)
        self.current_move_index = 0
        self.current_move_arr = []
        for i in range(len(path_dist)):
            self.current_move_arr.append(path_ang[i])
            self.current_move_arr.append(path_dist[i])
    
    def isMoveComplete(self):
        if(len(self.current_move_arr) == 0):
            return True
        return False

    def _calcMoves(self, path_arr):    
        arr = path_arr
        cnt = 0
        arr_direction_y = []
        arr_direction_x = []
        arr_angle = []
        for i in arr:
            if(cnt>0):
                y_dir = i[1]-arr[cnt-1][1]
                x_dir = i[0]-arr[cnt-1][0]
                arr_direction_y.append(y_dir)
                arr_direction_x.append(x_dir)
                angle = 0
                if(y_dir == 1):
                    angle = 0
                elif(y_dir == -1):
                    angle = 180
                elif(x_dir == 1):
                    angle = -90
                elif(x_dir == -1):
                    angle = 90
                arr_angle.append(angle)
            cnt+=1

        ang_prev = arr_angle[0]
        distance = 0
        arr_distance = []
        arr_angle_clean = []
        counter = 0
        for j in range(1,len(arr_angle)):
            counter+= 1
            if(arr_angle[j] == ang_prev and counter+1 <= len(arr_angle)):
                ang_prev = arr_angle[j]
                distance+=0.5   
                if(counter+1 == len(arr_angle)):
                    distance+=0.5
                    arr_distance.append(distance)
                    arr_angle_clean.append(ang_prev)
                    distance = 0
            else:    
                distance+=0.5
                arr_distance.append(distance)
                arr_angle_clean.append(ang_prev)
                if(counter+1 == len(arr_angle)):
                    arr_distance.append(0.5)
                    arr_angle_clean.append(arr_angle[j])
                distance = 0
                ang_prev = arr_angle[j]
        return arr_distance, arr_angle_clean