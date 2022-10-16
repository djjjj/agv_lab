#!/usr/bin/env python3
from typing import Tuple, List, Dict, Set
from heapq import heappush, heappop
import numpy as np
from scipy.spatial import KDTree

from .neighbour_table import NeighbourTable
from .grid import Grid
from .state import State


class Planner:

    def __init__(self, grid_size: int,
                       robot_radius: int,
                       static_obstacles: List[Tuple[int, int]]):
        self.grid_size = grid_size
        self.robot_radius = robot_radius
        np_static_obstacles = np.array(static_obstacles)
        self.static_obstacles = KDTree(np_static_obstacles)

        # Make the grid according to the grid size
        self.grid = Grid(grid_size, np_static_obstacles)
        # Make a lookup table for looking up neighbours of a grid
        self.neighbour_table = NeighbourTable(self.grid.grid)

    '''
    An admissible and consistent heuristic for A*
    '''
    @staticmethod
    def h(start: np.ndarray, goal: np.ndarray) -> int:
        return int(np.linalg.norm(start-goal, 1))  # L1 norm

    @staticmethod
    def l2(start: np.ndarray, goal: np.ndarray) -> int:
        return int(np.linalg.norm(start-goal, 2))  # L2 norm

    '''
    Check whether the nearest static obstacle is within radius
    '''
    def safe_static(self, grid_pos: np.ndarray) -> bool:
        _, nn = self.static_obstacles.query(grid_pos)
        return self.l2(grid_pos, self.static_obstacles.data[nn]) >= self.robot_radius

    '''
    Space-Time A*
    '''
    def plan(self, start: Tuple[int, int],
                   goal: Tuple[int, int],
                   dynamic_obstacles: Dict[int, Set[Tuple[int, int]]],
                   semi_dynamic_obstacles:Dict[int, Set[Tuple[int, int]]] = None,
                   max_iter:int = 500,
                   debug:bool = False) -> np.ndarray:
        # Prepare dynamic obstacles
        dynamic_obstacles = dict((k, np.array(list(v))) for k, v in dynamic_obstacles.items())
        # Assume dynamic obstacles are agents with same radius, distance needs to be 2*radius
        def safe_dynamic(grid_pos: np.ndarray, time: int) -> bool:
            nonlocal dynamic_obstacles
            return all(self.l2(grid_pos, obstacle) >= 2 * self.robot_radius
                       for obstacle in dynamic_obstacles.setdefault(time, np.array([])))

        # Prepare semi-dynamic obstacles, consider them static after specific timestamp
        if semi_dynamic_obstacles is None:
            semi_dynamic_obstacles = dict()
        else:
            semi_dynamic_obstacles = dict((k, np.array(list(v))) for k, v in semi_dynamic_obstacles.items())

        def safe_semi_dynamic(grid_pos: np.ndarray, time: int) -> bool:
            nonlocal semi_dynamic_obstacles
            for timestamp, obstacles in semi_dynamic_obstacles.items():
                flag = True
                if time >= timestamp:
                    flag = all(self.l2(grid_pos, obstacle) >= 2 * self.robot_radius for obstacle in obstacles)
                if not flag:
                    return False
            return True

        start = self.grid.snap_to_grid(np.array(start))
        goal = self.grid.snap_to_grid(np.array(goal))
        # Initialize the start state
        s = State(start, 0, 0, self.h(start, goal))
        open_set = [s]
        open_set_filter = {s}
        closed_set = set()

        # Keep track of parent nodes for reconstruction
        came_from = dict()
        iter_ = 0
        while open_set and iter_ < max_iter:
            iter_ += 1
            current_state = open_set[0]  # Smallest element in min-heap
            if current_state.pos_equal_to(goal):
                if debug:
                    print('STA*: Path found after {0} iterations'.format(iter_))
                return self.reconstruct_path(came_from, current_state)
            node = heappop(open_set)
            open_set_filter.remove(node)
            closed_set.add(node)
            epoch = current_state.time + 1
            for neighbour in self.neighbour_table.lookup(current_state.pos):
                neighbour_state = State(neighbour, epoch, current_state.g_score + 1, self.h(neighbour, goal))
                # Check if visited
                if neighbour_state in closed_set:
                    continue

                # Avoid obstacles
                if (
                    not self.safe_static(neighbour) \
                    or not safe_dynamic(neighbour, epoch) \
                    or not safe_semi_dynamic(neighbour, epoch)
                ) \
                and not np.array_equal(neighbour, goal):
                    continue
                # Add to open set
                if neighbour_state not in open_set_filter:
                    came_from[neighbour_state] = current_state
                    heappush(open_set, neighbour_state)
                    open_set_filter.add(neighbour_state)
        # return self.reconstruct_path(came_from, current_state)
        if debug:
            print('STA*: Open set is empty, no path found.')
        return np.array([])

    '''
    Reconstruct path from A* search result
    '''
    def reconstruct_path(self, came_from: Dict[State, State], current: State) -> np.ndarray:
        total_path = [current.pos]
        while current in came_from.keys():
            current = came_from[current]
            total_path.append(current.pos)
        return np.array(total_path[::-1])


if __name__ == '__main__':
    planner = Planner(
        1, 0.5, 
        [
            (4, 0), (18, 17), (4, 9), (5, 1), (8, 0), (17, 3), 
            (19, 0), (17, 12), (8, 9), (19, 9), (0, 5), (2, 2), 
            (0, 14), (13, 8), (6, 2), (7, 1), (13, 17), (18, 1), 
            (6, 11), (4, 2), (5, 3), (8, 2), (19, 2), (9, 1), (17, 14), 
            (8, 11), (19, 11), (0, 7), (2, 4), (11, 7), (0, 16), (13, 1), 
            (13, 10), (15, 7), (18, 3), (17, 7), (19, 4), (0, 0), (11, 0), 
            (17, 16), (0, 9), (11, 9), (15, 0), (13, 12), (15, 9), (18, 5), 
            (18, 14), (3, 1), (14, 1), (17, 0), (0, 2), (1, 3), (13, 5), (15, 2), 
            (16, 1), (13, 14), (15, 11), (18, 16), (3, 3), (5, 0), (14, 3), (17, 2), 
            (17, 11), (4, 11), (10, 8), (1, 5), (13, 7), (6, 1), (16, 3), (7, 0), 
            (13, 16), (18, 0), (5, 2), (4, 4), (17, 4), (9, 0), (10, 1), 
            (13, 0), (8, 13), (13, 9), (7, 2), (18, 2), (0, 18), (12, 1), 
            (8, 6), (1, 0), (13, 2), (8, 15), (10, 12), (13, 11), (0, 11), 
            (11, 11), (18, 4), (6, 8), (3, 0), (14, 0), (4, 8), (8, 8), (19, 8), 
            (1, 2), (0, 4), (2, 1), (8, 17), (16, 0), (0, 13), (2, 10), (6, 10), 
            (3, 2), (14, 2), (4, 1), (8, 1), (19, 1), (8, 10), (10, 7), (1, 4), (0, 6), 
            (2, 3), (16, 2), (19, 10), (0, 15), (2, 12), (6, 3), (6, 12), (3, 4), (4, 3), 
            (4, 12), (19, 3), (10, 0), (17, 15), (8, 12), (10, 9), (19, 12), (0, 8), (2, 5), 
            (11, 8), (16, 4), (0, 17), (15, 8), (12, 0), (21, 21), (17, 8), (8, 5), (19, 5), 
            (0, 1), (11, 1), (8, 14), (10, 11), (17, 17), (0, 10), (2, 7), (0, 19), (15, 1), 
            (13, 13), (12, 2), (18, 15), (4, 7), (17, 1), (17, 10), (8, 7), (19, 7), (1, 1), 
            (0, 3), (2, 0), (8, 16), (0, 12), (2, 9), (11, 12), (13, 6), (15, 3), (6, 0), (13, 15), 
            (15, 12), (6, 9)]
    )
    # print(planner.plan(
    #     (4, 1),
    #     (6, 1),
    #     {1: {(5, 1)}, 2: {(5, 1)}},
    #     {2: {(4, 1)}}
    # ))
    print(planner.plan(
        (2, 15),
        (6, 7),
        dynamic_obstacles={1: {(2, 14), (2, 15), (3, 15)}},
        semi_dynamic_obstacles={8: {(4, 10), (2, 8)}, 18: {(10, 10), (11, 10)}, 25: {(17, 9)}, 7: {(2, 11)}, 21: {(15, 10)}}
    ))