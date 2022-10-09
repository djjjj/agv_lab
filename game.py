from asyncore import dispatcher
from dis import dis
import json
import numpy

from multiprocessing import Value, Pool, Queue, Manager, Process

from .dispatcher import Dispatcher
from .element import AGV, Shelf, Cargo, Message
from .agv_controller import AGVController
from .cbs.planner import Planner
from .cbs.assigner import *
    


def main(data):
    static_obstacles = {(0, 0), (data['map_attr']['width'], data['map_attr']['height'])}
    agvs = {
        _['id']: AGV(_['id'], (_['x'], _['y']), _['payload'], _['cap']) 
        for _ in data['map_state']['agvs']
    }
    shelves = {
        _['id']: Shelf(_['id'], None, _['payload'], _['cap']) 
        for _ in data['map_state']['shelves']        
    }
    cargos = {
        _['id']: Cargo(_['id'], None, shelves[_['target']], _['weight']) 
        for _ in data['map_state']['cargos']        
    }
    
    for _ in data['map_state']['map']:
        if _['type'] == 'wall':
            static_obstacles.add((_['x'], _['y']))
        elif _['type'] == 'agv':
            agvs[_['id']].pos = (_['x'], _['y'])
        elif _['type'] == 'cargo':
            cargos[_['id']].pos = (_['x'], _['y'])
        elif _['type'] == 'shelf':
            shelves[_['id']].pos = (_['x'], _['y'])
            static_obstacles.add((_['x'], _['y']))

    static_obstacles = list(static_obstacles)
    agv_list = list(agvs.values())
    cargo_list = list(cargos.values())

    planner = Planner(1, 0.5, static_obstacles)
    dispatcher = Dispatcher(cargo_list)
    controller = AGVController(
        # data['map_attr']['max_steps'],
        100,
        agv_list
    )

    i = 0
    flag, step = controller.step()
    if flag:
        for agv in agv_list:
            agv.target
    standby = agv_list
    flag = True
    while True:
        if standby:
            dispatcher.dispatch(standby)
        if flag:
            starts, goals = [], []
            for _ in agv_list:
                starts.append(_.pos)
                if _.task == AGV.Task.STANDBY:
                    goals.append(_.pos)
                elif _.task == AGV.Task.DELIVERY:
                    goals.append(_.payload.target.pos)
                elif _.task == AGV.Task.PICKUP:
                    goals.append(_.target.pos)
            paths = planner.plan(starts, goals, assign=straight)
            controller.update_paths(paths)
        flag, step = controller.step()
        i += 1
        print(step)

if __name__ == '__main__':
    data = json.loads(open('/root/projects/agv_lab/game-maps.json', 'r').read())[3]
    main(data)