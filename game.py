# encoding: utf-8
import sys
import json
import os 

from multiprocessing import Value, Pool, Queue, Manager, Process
import logging
from typing import Any, Callable, List
import numpy

from dispatcher import Dispatcher
from element import AGV, Shelf, Cargo, Message, Wall
from agv_controller import AGVController
from cbs.planner import Planner
from cbs.assigner import *


STEP_Q = Manager().Queue()


def main(data):
    edge = numpy.zeros((data['map_attr']['height'], data['map_attr']['width']))
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
            edge[_['y'], _['x']] = 1
        elif _['type'] == 'agv':
            agvs[_['id']].pos = (_['x'], _['y'])
        elif _['type'] == 'cargo':
            cargos[_['id']].pos = (_['x'], _['y'])
        elif _['type'] == 'shelf':
            shelves[_['id']].pos = (_['x'], _['y'])
            static_obstacles.add((_['x'], _['y']))
            edge[_['y'], _['x']] = 1

    wall_edge = set()
    for i in range(edge.shape[0]):
        for j in range(edge.shape[1]):
            if edge[i, j] == 0:
                wall_edge.add(Wall((j - 1, i)))
                break
        for j in range(edge.shape[1] - 1, -1, -1):
            if edge[i, j] == 0:
                wall_edge.add(Wall((j + 1, i)))
                break
    for j in range(edge.shape[1]):
        for i in range(edge.shape[0]):
            if edge[i, j] == 0:
                wall_edge.add(Wall((j, i - 1)))
                break
        for i in range(edge.shape[0] - 1, -1, -1):
            if edge[i, j] == 0:
                wall_edge.add(Wall((j, i + 1)))
                break
    wall_edge = list(wall_edge)

    static_obstacles = list(static_obstacles)
    agv_list = list(agvs.values())
    cargo_list = list(cargos.values())

    planner = Planner(1, 0.5, static_obstacles)
    dispatcher = Dispatcher(cargo_list)
    dispatcher2 = Dispatcher(wall_edge)
    controller = AGVController(
        # data['map_attr']['max_steps'],
        100,
        agv_list
    )
    standby = []
    flag = True
    starts, goals = [], []
    max_step = data['map_attr']['max_steps']
    cnt_step = 0
    while cnt_step <= max_step:
        standby = [_ for _ in agv_list if _.task == AGV.Task.STANDBY]
        if len(standby) == len(agv_list) and len(dispatcher.cargos) == 0:
            break
        if standby:
            dispatcher.dispatch(standby)

        if flag:
            starts, goals = [], []
            standby = [_ for _ in agv_list if _.task == AGV.Task.STANDBY]
            if standby:
                a, b = dispatcher2.standby(standby)
            for _ in agv_list:
                starts.append(_.pos)
                if _.task == AGV.Task.STANDBY:
                    goals.append(_.target.pos)
                elif _.task == AGV.Task.DELIVERY:
                    goals.append(_.payload.target.pos)
                elif _.task == AGV.Task.PICKUP:
                    goals.append(_.target.pos)
            paths = planner.plan(starts, goals, assign=straight, max_iter=100, low_level_max_iter=100)
            controller.update_paths(paths)
        flag, step = controller.step()
        if step:
            cnt_step += 1
            STEP_Q.put(step)


def start_online():
    import marathon
    from marathon import Submission,Map

    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)
    logger.addHandler(logging.StreamHandler(stream=sys.stdout))

    sub = marathon.new_submission()
    maps = sub.maps()

    for m in maps:
        print(m.id)
        m.start_game()
        p = Process(
            target=main,
            args=({
                'map_attr': m.get_map_attr(),
                'map_state': m.get_map_state()
            },)
        )
        p.start()

        while True:
            try:
                step = STEP_Q.get(timeout=2)
            except :
                p.terminate()
                break
            m.step(step)
            logger.info("运行gameId: {}， 总步数为：{}，是否完成比赛：{}".format(m.get_game_id(),m.get_steps(),m.get_done()))
            logger.info("当前状态：")
            logger.info(m.map_state)
            print(sub.finish())


def start_offline():
    maps = json.loads(open(os.path.join(os.path.dirname(__file__), 'game-maps.json')).read())

    for m in maps:
        if m['map_id'] != 'w1':
            continue
        print(m['map_id'])
        # main(
        #     {
        #         'map_attr': m['map_attr'],
        #         'map_state': m['map_state']
        #     }
        # )
        p = Process(
            target=main,
            args=({
                'map_attr': m['map_attr'],
                'map_state': m['map_state']
            },)
        )
        p.start()
        while True:
            try:
                step = STEP_Q.get(timeout=2)
            except :
                # p.terminate()
                break
            print(step)


if __name__ == '__main__':
    start_offline()