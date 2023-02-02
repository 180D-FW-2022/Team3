from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
import numpy as np

grid_loaded = np.ones((20,20))
grid = Grid(matrix=grid_loaded)

start = grid.node(0, 0)
end = grid.node(19, 19)

finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
path, runs = finder.find_path(start, end, grid)

print(path)