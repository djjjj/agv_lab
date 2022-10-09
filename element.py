from asyncio import Task
from typing import Tuple, List, Set
from enum import Enum

class Element(object):

    def __init__(self, id: int, pos: Tuple[int, int], type: str) -> None:
        self.id = id  
        self.pos = pos
        self.type = type

    def __hash__(self) -> int:
        return hash(self.type + str(self.id))

    def __str__(self) -> str:
        return self.type + str(self.id)

class Shelf(Element):

    def __init__(self, 
                id: int, 
                pos: Tuple[int, int], 
                payload: Set, cap: int, 
                # ports: List[Tuple[int, int]]
            ) -> None:
        super().__init__(id, pos, "S")
        self.payload = payload
        self.cap = cap
        # self.ports = ports 

    def loading(self, cargo):
        assert len(self.payload) + 1 <= self.cap, "no space"
        self.payload.add(cargo)

    def unloading(self):
        assert len(self.payload) > 0, "no cargos"
        return self.payload.pop()

class Cargo(Element):

    def __init__(self, id: int, pos: Tuple[int, int], target: Shelf, weight: int) -> None:
        super().__init__(id, pos, 'C')
        self.target = target
        self.weight = weight

class AGV(Element):

    class Task(Enum):

        DELIVERY = 0
        PICKUP = 1
        STANDBY = -1

    class Action(Enum):
        DELIVERY = 0
        PICKUP = 1
        MOVE = 2
        STAY = 3

    class Direct(Enum):

        UP = (0, -1)
        DOWN = (0, 1)
        RIGHT = (1, 0)
        LEFT = (-1, 0)
        NONE = (0, 0)

    def __init__(self, id: int, pos: Tuple[int, int], payload: Cargo, cap: int ) -> None:
        super().__init__(id, pos, 'A')
        self.payload = payload
        self.cap = cap 
        self.task = self.Task.DELIVERY if payload is not None else self.Task.STANDBY
        self.target = None if payload is None else payload.target

    def pickup(self, direct: Direct):
        self.payload = self.target
        self.task = self.Task.DELIVERY
        self.target = None
        return {"type": self.Action.PICKUP.name, "dir": direct.name}

    def delivery(self, direct: Direct):
        self.payload = None
        self.task = self.Task.STANDBY
        self.target = None
        return {"type": self.Action.DELIVERY.name, "dir": direct.name}

    def move(self, direct: Direct):
        self.pos = (self.pos[0] + direct.value[0], self.pos[1] + direct.value[1])
        return {"type": self.Action.MOVE.name, "dir": direct.name}

    def stay(self):
        return {"type": self.Action.STAY.name}
    

class Message:

    def __init__(self, agv_id: int, pos: Tuple[int, int]) -> None:
        self.agv_id = agv_id
        self.pos = pos
