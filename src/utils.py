"""General utilities for grid-map handling.

Includes functions for loading/binarizing image terrains and 
synthesizing visual matplotlib figures for performance reports.
"""

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def load_map(file_path):
    """Load and binarize an image map.
    
    Returns:
        2D numpy array where 0 is free space and 1 is occupied obstacle.
    """
    image = Image.open(file_path).convert('L')
    grid_map = np.array(image.getdata()).reshape((image.size[1], image.size[0])) / 255.0
    grid_map[grid_map > 0.5] = 1
    grid_map[grid_map <= 0.5] = 0
    # Invert so 0 = free, 1 = occupied
    grid_map = (grid_map * -1) + 1
    return grid_map


def plot_path_on_map(grid_map, trajectory, start, goal, title, save_path=None):
    """Plot the grid map alongside start, goal, and the extracted path.
    
    If trajectory is empty or False, the path line is skipped.
    """
    plt.figure(figsize=(8, 8))
    plt.matshow(grid_map, fignum=0, cmap='gray_r')
    
    plt.plot(start[1], start[0], 'go', markersize=10, label='Start')
    plt.plot(goal[1], goal[0], 'r*', markersize=10, label='Goal')
    
    if trajectory:
        traj_x = [point[0] for point in trajectory]
        traj_y = [point[1] for point in trajectory]
        # In matshow plotting vectors: X on plot is Col (Y in array), Y on plot is Row (X in array)
        plt.plot(traj_y, traj_x, 'b-', linewidth=2, label='Path')
        
    plt.title(title, pad=20)
    plt.legend()
    
    if save_path:
        plt.savefig(save_path, bbox_inches='tight')
    plt.close()
