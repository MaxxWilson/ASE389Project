import numpy as np


class MetaSingleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(MetaSingleton,
                                        cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class AtlasStateProvider(metaclass=MetaSingleton):
    def __init__(self, robot):
        self._robot = robot
        self._nominal_joint_pos = dict()

    @property
    def nominal_joint_pos(self):
        return self._nominal_joint_pos

    @nominal_joint_pos.setter
    def nominal_joint_pos(self, val):
        assert self._robot.n_a == len(val.keys())
        self._nominal_joint_pos = val
