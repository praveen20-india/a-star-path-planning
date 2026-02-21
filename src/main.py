"""Main pipeline script for A* Path Planning.

Executes continuous testing scenarios traversing multiple maps,
evaluates differing grid connectivity schemas, and saves performance summaries.
"""

import os
from a_star import AStarPlanner
from utils import load_map, plot_path_on_map
from heuristics import precompute_heuristic_map


def run_scenario(map_name, map_file, start, goal, save_dir, results_file):
    """Run A* search evaluation over a single configured map context.

    Saves plotted image graphs tracking extracted connectivity paths.
    Appends raw benchmark heuristics identifying search limits to results_file.
    """
    print(f"Running scenario: {map_name}...")
    
    os.makedirs(save_dir, exist_ok=True)
    os.makedirs(os.path.dirname(results_file), exist_ok=True)

    grid_map = load_map(map_file)

    # Establish strictly admissible euclidean precomputations
    h_map = precompute_heuristic_map(grid_map.shape, goal, metric="euclidean")

    with open(results_file, 'a') as f:
        f.write(f"--- Scenario: {map_name} ---\n")

    for connectivity in [4, 8]:
        planner = AStarPlanner(grid_map, start, goal, h_map, connectivity)
        
        path, total_cost = planner.run_a_star()

        # Save output graphic
        save_path = os.path.join(save_dir, f"{map_name}_conn_{connectivity}.png")
        title = f"A* Path ({map_name} - {connectivity}-Connected)"
        plot_path_on_map(grid_map, path, start, goal, title, save_path)

        # Log completion benchmarks
        with open(results_file, 'a') as f:
            if path:
                f.write(f"Connectivity {connectivity} - Cost: {total_cost:.2f}, Nodes in Path: {len(path)}\n")
            else:
                f.write(f"Connectivity {connectivity} - Failure: Disconnected Domain.\n")

    with open(results_file, 'a') as f:
        f.write("\n")


def main():
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_dir = os.path.join(base_dir, 'data')
    save_dir = os.path.join(base_dir, 'report', 'result_images')
    results_file = os.path.join(base_dir, 'results', 'performance_summary.txt')

    # Clear previous pipeline iterations
    if os.path.exists(results_file):
        os.remove(results_file)

    scenarios = [
        {'name': 'map0_goal1', 'file': 'map0.png', 'start': [10, 10], 'goal': [40, 110]},
        {'name': 'map0_goal2', 'file': 'map0.png', 'start': [10, 10], 'goal': [70, 90]},
        {'name': 'map1', 'file': 'map1.png', 'start': [60, 60], 'goal': [60, 90]},
        {'name': 'map2', 'file': 'map2.png', 'start': [8, 31], 'goal': [38, 139]},
        {'name': 'map3', 'file': 'map3.png', 'start': [50, 90], 'goal': [375, 375]}
    ]

    for sc in scenarios:
        run_scenario(
            map_name=sc['name'],
            map_file=os.path.join(data_dir, sc['file']),
            start=sc['start'],
            goal=sc['goal'],
            save_dir=save_dir,
            results_file=results_file
        )
        
    print(f"Pipeline execution completed. Results appended to {results_file}")


if __name__ == "__main__":
    main()
