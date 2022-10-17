#!/usr/bin/env python3
import time 
from typing import List, Tuple, Dict, Callable, Set
import multiprocessing as mp
from heapq import heappush, heappop
from itertools import combinations
from copy import deepcopy
import numpy as np

# The low level planner for CBS is the Space-Time A* planner
# https://github.com/GavinPHR/Space-Time-AStar
from .stastar.planner import Planner as STPlanner
from .constraint_tree import CTNode
from .constraints import Constraints
from .agent import Agent
from .assigner import *


class Planner:

    def __init__(self, grid_size: int,
                       robot_radius: int,
                       static_obstacles: List[Tuple[int, int]]):
        self.robot_radius = robot_radius
        self.st_planner = STPlanner(grid_size, robot_radius, static_obstacles)

    '''
    You can use your own assignment function, the default algorithm greedily assigns
    the closest goal to each start.
    '''
    def plan(self, starts: List[Tuple[int, int]],
                   goals: List[Tuple[int, int]],
                   assign:Callable = min_cost,
                   max_iter:int = 200,
                   low_level_max_iter:int = 100,
                   max_process:int = 10,
                   debug:bool = False) -> np.ndarray:
        self.low_level_max_iter = low_level_max_iter
        self.debug = debug

        # Do goal assignment
        self.agents = assign(starts, goals)
        constraints = Constraints()
        # Compute path for each agent using low level planner
        solution = {}
        paths = []
        for agent in self.agents:
            path = self.calculate_path(agent, constraints, None)
            if len(path) == 0:
                path = np.array([agent.start])
            paths.append(path)
        index = self.sniff(paths)

        for agent, path in zip(self.agents, paths):
            solution[agent] = path[:index]

        open = []
        if all(len(path) != 0 for path in solution.values()):
            # Make root node
            node = CTNode(constraints, solution)
            # Min heap for quick extraction
            open.append(node)
        manager = mp.Manager()
        iter_ = 0
        start = time.time()
        while open and iter_ < max_iter:
            iter_ += 1
            # results = manager.list([])
            # processes = []
            # Default to 10 processes maximum
            for _ in range(max_process if len(open) > max_process else len(open)):
            #     p = mp.Process(target=self.search_node, args=[heappop(open), results])
            #     processes.append(p)
            #     p.start()
            # for p in processes:
            #     p.join()
                node = heappop(open)
#                 print(
#                     node.constraints,
#                     node.solution,
#                     node.cost, "start"
#                 )
                result = self.search_node(node)
                # print(result)
                # if result[0] is not None:
                #     print(
                #         result[0].constraints,
                #         result[0].solution,
                #         result[0].cost, "left"
                #     )
                # if result[1] is not None:
                #     print(
                #         result[1].constraints,
                #         result[1].solution,
                #         result[1].cost, "right"
                #     )
                # input("")
            # for result in results:
                if len(result) == 1:
                    if debug:
                        print('CBS_MAPF: Paths found after about {0} iterations'.format(4 * iter_))
                    return result[0]
                if result[0]:
                    heappush(open, result[0])
                if result[1]:
                    heappush(open, result[1])
        if debug:
            print('CBS-MAPF: Open set is empty, no paths found.')
        return np.array([])

    '''
    Abstracted away the cbs search for multiprocessing.
    The parameters open and results MUST BE of type ListProxy to ensure synchronization.
    '''
    def search_node(self, best: CTNode):
        # agent_i, agent_j, time_of_conflict = self.validate_paths(self.agents, best)
        # If there is not conflict, validate_paths returns (None, None, -1)
        agent_i, agent_j, agent_i_constraint, agent_j_constraint = self.validate_paths(self.agents, best)
        if agent_i_constraint is None:
            # results.append((self.reformat(self.agents, best.solution),))
            # return
            return (self.reformat(self.agents, best.solution),)
        # Calculate new constraints
        # agent_i_constraint = self.calculate_constraints(best, agent_i, agent_j, time_of_conflict)
        # agent_j_constraint = self.calculate_constraints(best, agent_j, agent_i, time_of_conflict)
#         print(agent_i_constraint, agent_j_constraint, 123)
        # Calculate new paths
        agent_i_path = self.calculate_path(agent_i,
                                           agent_i_constraint,
                                           self.calculate_goal_times(best, agent_i, self.agents))
        agent_j_path = self.calculate_path(agent_j,
                                           agent_j_constraint,
                                           self.calculate_goal_times(best, agent_j, self.agents))
#         print(self.calculate_goal_times(best, agent_i, self.agents), self.calculate_goal_times(best, agent_j, self.agents))
#         print(agent_i_path, agent_j_path, 'pathsss')
        # Replace old paths with new ones in solution
        solution_i = best.solution
        solution_j = deepcopy(best.solution)
        solution_i[agent_i] = agent_i_path
        solution_j[agent_j] = agent_j_path
        node_i = None
        if all(len(path) != 0 for path in solution_i.values()):
            node_i = CTNode(agent_i_constraint, solution_i)
        node_j = None
        if all(len(path) != 0 for path in solution_j.values()):
            node_j = CTNode(agent_j_constraint, solution_j)
        # results.append((node_i, node_j))
        return (node_i, node_j)

    '''
    Pair of agent, point of conflict
    '''
    def validate_paths(self, agents, node: CTNode):
        solution = node.solution
        for agent_i, agent_j in combinations(agents, 2):
            for idx in range(min(len(solution[agent_i]), len(solution[agent_j])) - 1):
                pt1, pt2 = solution[agent_i][idx], solution[agent_i][idx+1]
                pt3, pt4 = solution[agent_j][idx], solution[agent_j][idx+1]
                if np.array_equal(pt2, pt4):
                    return (
                        agent_i, agent_j,
                        node.constraints.fork(
                            agent_i, {idx+1: {tuple(pt2)}}),
                        node.constraints.fork(
                            agent_j, {idx+1: {tuple(pt2)}}),
                        )
                elif (np.array_equal(pt1, pt4) and np.array_equal(pt2, pt3)):
                    return (
                    agent_i, agent_j,
                    node.constraints.fork(
                        agent_i, {idx+1: {tuple(pt2),}}),
                    node.constraints.fork(
                        agent_j, {idx+1: {tuple(pt4), tuple(pt3)}},),
                    )
                elif np.array_equal(pt2, pt3):
                    return (
                    agent_i, agent_j,
                    node.constraints.fork(
                        agent_i, {idx+1: {tuple(pt3),}}),
                    node.constraints.fork(
                        agent_j, {idx+1: {tuple(pt4), tuple(pt3)}}),
                    )
                elif np.array_equal(pt1, pt4):
                    return (
                        agent_i, agent_j,
                        node.constraints.fork(
                            agent_i, {idx+1: {tuple(pt1), tuple(pt2)}}),
                        node.constraints.fork(
                            agent_j, {idx+1: {tuple(pt1)}},),
                    )
        return (None, None, None, None, )

    def safe_distance(self, solution: Dict[Agent, np.ndarray], agent_i: Agent, agent_j: Agent) -> int:
        for idx in range(min(len(solution[agent_i]), len(solution[agent_j])) - 1):
            pt1, pt2 = solution[agent_i][idx], solution[agent_i][idx+1]
            pt3, pt4 = solution[agent_j][idx], solution[agent_j][idx+1]
            if np.array_equal(pt1, pt3): return idx 
            elif np.array_equal(pt2, pt4): return idx + 1
            elif np.array_equal(pt1, pt4) and np.array_equal(pt2, pt3): return idx + 1
        # for idx, (point_i, point_j) in enumerate(zip(solution[agent_i], solution[agent_j])):
        #     if self.dist(point_i, point_j) > 2*self.robot_radius:
        #         continue
        #     elif self.dist(point_i, point_j) == 2*self.robot_radius:
        #         return
        #     return idx
        # return -1

    @staticmethod
    def dist(point1: np.ndarray, point2: np.ndarray) -> int:
        return int(np.linalg.norm(point1-point2, 2))  # L2 norm

    def calculate_constraints(self, node: CTNode,
                                    constrained_agent: Agent,
                                    unchanged_agent: Agent,
                                    time_of_conflict: int) -> Constraints:
        contrained_path = node.solution[constrained_agent]
        unchanged_path = node.solution[unchanged_agent]
        pivot = unchanged_path[time_of_conflict]
        conflict_end_time = time_of_conflict
        try:
            while self.dist(contrained_path[conflict_end_time], pivot) < 2*self.robot_radius:
                conflict_end_time += 1
        except IndexError:
            pass
        return node.constraints.fork(constrained_agent, tuple(pivot.tolist()), time_of_conflict, conflict_end_time)

    def calculate_goal_times(self, node: CTNode, agent: Agent, agents: List[Agent]):
        solution = node.solution
        goal_times = dict()
        for other_agent in agents:
            if other_agent == agent:
                continue
            time = len(solution[other_agent]) - 1
            goal_times.setdefault(time, set()).add(tuple(solution[other_agent][time]))
        return goal_times

    '''
    Calculate the paths for all agents with space-time constraints
    '''
    def calculate_path(self, agent: Agent, 
                       constraints: Constraints, 
                       goal_times: Dict[int, Set[Tuple[int, int]]]) -> np.ndarray:
        return self.st_planner.plan(agent.start, 
                                    agent.goal, 
                                    constraints.setdefault(agent, dict()), 
                                    semi_dynamic_obstacles=goal_times,
                                    max_iter=self.low_level_max_iter, 
                                    debug=self.debug)

    '''
    Reformat the solution to a numpy array
    '''
    @staticmethod
    def reformat(agents: List[Agent], solution: Dict[Agent, np.ndarray]):
        solution = Planner.pad(solution)
        reformatted_solution = []
        for agent in agents:
            reformatted_solution.append(solution[agent])
        return np.array(reformatted_solution)

    '''
    Pad paths to equal length, inefficient but well..
    '''
    @staticmethod
    def pad(solution: Dict[Agent, np.ndarray]):
        max_ = max(len(path) for path in solution.values())
        for agent, path in solution.items():
            if len(path) == max_:
                continue
            padded = np.concatenate([path, np.array(list([path[-1]])*(max_-len(path)))])
            solution[agent] = padded
        return solution

    def sniff(self, paths):
        buckets = []
        for i in range((max([len(_) for _ in paths]) - 1)):
            buckets.append(dict())
        for i, j in combinations(range(len(paths)), 2):
            path_i, path_j = paths[i], paths[j]
            for idx in range(min(len(path_i), len(path_j)) - 1):
                pt1, pt2 = tuple(path_i[idx]), tuple(path_i[idx+1])
                pt3, pt4 = tuple(path_j[idx]), tuple(path_j[idx+1])
                if np.array_equal(pt2, pt4):
                    if pt2 not in buckets[idx]:
                        buckets[idx][pt2] = set()
                    buckets[idx][pt2].update([i, j])
                elif (np.array_equal(pt1, pt4) and np.array_equal(pt2, pt3)):
                    if pt1 not in buckets[idx]:
                        buckets[idx][pt1] = set()
                    buckets[idx][pt1].update([i, j])
                    if pt2 not in buckets[idx]:
                        buckets[idx][pt2] = set()
                    buckets[idx][pt2].update([i, j])
                elif np.array_equal(pt2, pt3):
                    if pt3 not in buckets[idx]:
                        buckets[idx][pt3] = set()
                    buckets[idx][pt3].update([i, j])
                elif np.array_equal(pt1, pt4):
                    if pt4 not in buckets[idx]:
                        buckets[idx][pt4] = set()
                    buckets[idx][pt4].update([i, j])
        i = 0
        while i < len(buckets):
            if len([__ for __ in buckets[i].values() if len(__) > 1]) != 0:
                break
            i += 1
        if i > 1:
            return i + 1
        return 3
        total = 0
        for i, _ in enumerate(buckets):
            cnt = len([__ for __ in _.values() if len(__) > 1])
            total += cnt 
            if total >= 8:
                return min(max(i, 2), 5)
        return max(len(buckets), 2) 

if __name__ == '__main__':
    planner = Planner(
        1, 0.5,
        [
            (0, 0), (9, 2),
            (1, 0), (2, 0), (3, 0), (4, 0),
            (6, 0), (7, 0), (8, 0), (9, 0),
        ],
    )
    print(planner.plan(
        [(4, 1), (6, 1)],
        [(6, 1), (4, 1)],
        assign=straight
    ))

