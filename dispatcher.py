import numpy
import time 
from typing import List 

from .element import AGV, Cargo
from scipy.optimize import linear_sum_assignment
from multiprocessing import Queue


class Dispatcher(object):

    def __init__(self, cargo_list: List[Cargo]) -> None:
        self.cargos = cargo_list

    def dispatch(self, agvs: List[AGV]):
        cargo_pos_list = [_.pos for _ in self.cargos]
        agv_pos_list = [_.pos for _ in agvs]
        starts_idx, goals_idx = self._min_dist(agv_pos_list, cargo_pos_list) 
        starts, golas = [], []
        for i, j in zip(starts_idx, goals_idx):
            starts.append(agvs[i])
            agvs[i].task = AGV.Task.PICKUP
            agvs[i].target = self.cargos[j]
            golas.append(self.cargos[j])
        for cargo in golas:
            self.cargos.remove(cargo)
        return starts, golas

    def _min_dist(self, starts, goals):
        sqdist = lambda x, y: (x[0]-y[0])**2 + (x[1]-y[1])**2
        cost_mtx = numpy.zeros((len(starts), len(goals)), dtype=numpy.int16)
        for i, start in enumerate(starts):
            for j, goal in enumerate(goals):
                cost_mtx[i, j] = sqdist(start, goal)

        return linear_sum_assignment(cost_mtx) 
        

if __name__ == '__main__':
    d = Dispatcher([
        AGV(0, (0, 0), None, None),
        AGV(1, (8, 9), None, None),
        AGV(1, (8, 92), None, None),
        AGV(1, (81, 19), None, None),
    ])

    cargos = [
        Cargo(0, (10, 22), None, None), 
        Cargo(1, (2, 3), None, None),
        Cargo(2, (81, 19), None, None),
    ]

    print(d.dispatch(cargos))