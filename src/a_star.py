"""A* Path Planning core evaluation module.

Applies the A* search procedure f(n) = g(n) + h(n) using priority 
queues to assure optimally extracted trajectories through grid spaces.
"""

import numpy as np
import heapq as hq
from heuristics import euclidean_distance


class AStarPlanner:
    """Core A* planner object for grid-based trajectory generation."""

    def __init__(self, grid_map, start, goal, heuristic_map, connectivity=4):
        """Initialize the planner environment.

        Args:
            grid_map: 2D numpy array (0 = free, 1 = obstacle).
            start: List [row, col] starting configuration.
            goal: List [row, col] target destination.
            heuristic_map: Precomputed 2D h(n) array.
            connectivity: Integer 4 or 8 defining neighbor expansion rules.
        """
        self.grid_map = grid_map
        self.start = list(start)
        self.goal = list(goal)
        self.h_map = heuristic_map
        self.connectivity = connectivity

    def is_valid_cell(self, next_step):
        """Verify grid bounds and obstacle collisions."""
        if (
            next_step[0] >= self.grid_map.shape[0]
            or next_step[0] < 0
            or next_step[1] >= self.grid_map.shape[1]
            or next_step[1] < 0
            or self.grid_map[next_step[0]][next_step[1]] == 1
        ):
            return False
        return True

    def run_a_star(self):
        """Execute the A* search.

        Returns:
            Tuple (path, total_cost). Returns (False, None) if no valid 
            path to the goal exists.
        """
        if self.connectivity == 4:
            move_list = [[1, 0], [0, 1], [-1, 0], [0, -1]]
        elif self.connectivity == 8:
            move_list = [
                [0, 1], [1, 1], [1, 0], [1, -1],
                [0, -1], [-1, -1], [-1, 0], [-1, 1],
            ]
        else:
            raise ValueError(f"Connectivity {self.connectivity} must be 4 or 8.")

        camefrom = {f"{self.start}": None}

        # Arrays to hold exact cost (g) and estimated total cost (f)
        g_score = np.full(self.grid_map.shape, np.inf)
        g_score[self.start[0]][self.start[1]] = 0

        f_score = np.full(self.grid_map.shape, np.inf)
        f_score[self.start[0]][self.start[1]] = self.h_map[self.start[0]][self.start[1]]

        # Priority Queue: stores tuples of (f_score, [row, col] position)
        open_set = [(f_score[self.start[0]][self.start[1]], self.start)]
        
        # Track explored items using a fast set of stringified coordinates
        close_set_fast = set()
        open_set_fast = {f"{self.start}"}

        while len(open_set) != 0:
            current = hq.heappop(open_set)
            current_pos = current[1]
            
            open_set_fast.discard(f"{current_pos}")
            close_set_fast.add(f"{current_pos}")

            # Check if destination reached
            if current_pos == self.goal:
                total_cost = current[0]
                return self.reconstruct_path(camefrom, current_pos), total_cost

            # Explore valid neighbors
            for move in move_list:
                next_step = [current_pos[0] + move[0], current_pos[1] + move[1]]

                if not self.is_valid_cell(next_step):
                    continue

                if f"{next_step}" in close_set_fast:
                    continue

                # The cost of movement to exactly adjacent neighbors formulation
                step_cost = euclidean_distance(current_pos, next_step)
                tentative_g_score = g_score[current_pos[0]][current_pos[1]] + step_cost

                # Add neighbor if novel, or update if a strictly shorter path was found
                if f"{next_step}" not in open_set_fast or tentative_g_score < g_score[next_step[0]][next_step[1]]:
                    camefrom[f"{next_step}"] = current_pos
                    g_score[next_step[0]][next_step[1]] = tentative_g_score
                    f_score[next_step[0]][next_step[1]] = tentative_g_score + self.h_map[next_step[0]][next_step[1]]

                    if f"{next_step}" not in open_set_fast:
                        hq.heappush(open_set, (f_score[next_step[0]][next_step[1]], next_step))
                        open_set_fast.add(f"{next_step}")
                    else:
                        # Update priority via heap rebuild if cost strictly improved
                        for idx, item in enumerate(open_set):
                            if item[1] == next_step:
                                open_set[idx] = (f_score[next_step[0]][next_step[1]], next_step)
                                break
                        hq.heapify(open_set)

        return False, None

    def reconstruct_path(self, camefrom, current):
        """Ascend the parent pointer chain to reassemble the sequential path."""
        total_path = [current]
        while camefrom[f"{current}"] is not None:
            current = camefrom[f"{current}"]
            total_path.insert(0, current)
        return total_path
