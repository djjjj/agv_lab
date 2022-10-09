import numpy
from typing import List
from multiprocessing import Queue, Value

from .element import AGV

class AGVController(object):

    def __init__(self, max_step: int, agv_list: List[AGV]) -> None:
        self.paths = numpy.zeros(
            (len(agv_list), max_step, 2),
            dtype=numpy.int16,
            order='F'
        )
        for i, agv in enumerate(agv_list):
            self.paths[i] = [list(agv.pos)] * max_step

        self.agvs = agv_list
    
    def step(self):
        if self.paths.shape[1] <= 1:
            return[]
        actions = []
        diff = self.paths[:, 1] - self.paths[:, 0]
        flag = False
        for i, agv in enumerate(self.agvs):
            direct = AGV.Direct(tuple(diff[i]))
            if direct == AGV.Direct.NONE:
                actions.append(agv.stay())
            elif agv.task == AGV.Task.PICKUP and numpy.array_equal(self.paths[i, 1], agv.target.pos):
                actions.append(agv.pickup(direct))
                self.paths[i, 1:] = self.paths[i, 0]
                flag = True
            elif agv.task == AGV.Task.DELIVERY and numpy.array_equal(self.paths[i, 1], agv.payload.target.pos):
                actions.append(agv.delivery(direct))
                self.paths[i, 1:] = self.paths[i, 0]
                flag = True
            else:
                actions.append(agv.move(direct))
        self.paths = self.paths[:,1:]
        return flag, actions

    def update_paths(self, paths: numpy.ndarray):
        self.paths[:, 0: paths.shape[1]] = paths
        self.paths[:, paths.shape[1]:] = numpy.tile(
            paths[:, -1:],
            (1, self.paths.shape[1] - paths.shape[1], 1),
        )
        
        
if __name__ == '__main__':
    c = AGVController(
        10,
        [AGV(0, (0, 0), None, 1), AGV(0, (1, 1), None, 1), AGV(0, (2, 2), None, 1)]
    )
    step = c.step()
    print(step)
    c.paths = numpy.array([
        [[0, 0], [0, 1], [0, 2]],
        [[1, 1], [1, 2], [1, 3]],
        [[2, 2], [2, 3], [3, 3]]
    ])
    step = c.step()
    print(step)
    step = c.step()
    print(step)
    step = c.step()
    print(step)