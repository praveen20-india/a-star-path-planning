"""Heuristic functions for A* Path Planning.

Contains various distance estimations serving as admissible heuristics 
(h(n)) for guided grid-based search space evaluation. 
"""

import numpy as np


def euclidean_distance(p1, p2):
    """Compute the Euclidean distance metric between two points.
    
    Admissible for any spatial grid search as it represents the 
    straight-line optimal path.
    """
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def manhattan_distance(p1, p2):
    """Compute the Manhattan distance metric between two points.
    
    Strictly admissible for 4-connected grid spaces.
    """
    return np.abs(p1[0] - p2[0]) + np.abs(p1[1] - p2[1])


def chebyshev_distance(p1, p2):
    """Compute the Chebyshev distance metric between two points.
    
    Strictly admissible for 8-connected grid spaces where diagonal 
    movement shares equal cost with cardinal movement.
    """
    return max(np.abs(p1[0] - p2[0]), np.abs(p1[1] - p2[1]))


def precompute_heuristic_map(grid_shape, goal, metric="euclidean"):
    """Precompute a full 2D array of heuristic values to the goal.

    Args:
        grid_shape: Tuple representing map dimensions (rows, cols).
        goal: The [row, col] coordinates of the target destination.
        metric: Distance String ('euclidean', 'manhattan', 'chebyshev')

    Returns:
        2D numpy array holding h(n) values for every grid cell.
    """
    h_map = np.zeros(shape=grid_shape)
    
    if metric == "euclidean":
        dist_func = euclidean_distance
    elif metric == "manhattan":
        dist_func = manhattan_distance
    elif metric == "chebyshev":
        dist_func = chebyshev_distance
    else:
        raise ValueError(f"Unknown heuristic metric: {metric}")

    for i, j in np.ndindex(h_map.shape):
        h_map[i][j] = dist_func([i, j], goal)
        
    return h_map
