from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)

import numpy as np
import random


class RobotDriver(object):
    def __init__(self, robot_id, map, initial_localization=None):
        if initial_localization is None:
            initial_localization = random.choice(
                [tuple(e) for e in np.argwhere(map == MapObject.EMPTY)])
        self._initial_localization = initial_localization
        self._robot_id = robot_id
        self._path = [initial_localization]
        self._notification = RobotNotification.NONE
        self._map = np.copy(map)
        self._status = (0, RobotStatus.STOP)
        self._obstacles = []
        self._humans = []
        self._movement_iterations = 0

    def read_and_clear_obstacles(self):
        res = self._obstacles
        self._obstacles = []
        return res

    def read_and_clear_humans(self):
        res = self._humans
        self._humans = []
        return res

    def get_notify(self):
        return self._notification

    def reset_notify(self):
        self._notification = RobotNotification.NONE

    def _set_notify(self, notification):
        self._notification = notification

    def get_position(self):
        return self._path[self._status[0]]

    def _set_position(self):
        return self._path[self._status[0]]

    def get_status(self):
        return self._status

    def set_status(self, state):
        self._status = (self._status[0], state)

    def set_step(self, step):
        self._status = (step, self._status[1])

    def get_id(self):
        return self._robot_id

    def set_path(self, path):
        self._path = path

    def _arrived_event(self):
        self._notification = RobotNotification.ARRIVED

    def _want_run_event(self):
        self._notification = RobotNotification.WANT_RUN

    def _found_human_event(self):
        self._notification = RobotNotification.FOUND_HUMAN

    def _found_obstacle_event(self):
        self._notification = RobotNotification.FOUND_OBSTACLE

    def _none_event(self):
        self._notification = RobotNotification.NONE

    def detect_obstacle(self):  # wykrycie przeszkody z mapy rzeczywistej
        floor, x, y = self._path[self._status[0]]
        floors, x_max, y_max = self._map.shape
        self._notification = RobotNotification.NONE
        for x_i in range(3):
            for y_i in range(3):
                new_x = x - 1 + x_i
                new_y = y - 1 + y_i
                if 0 <= new_x < x_max and 0 <= new_y < y_max:
                    if self._map[floor, new_x, new_y] == MapObject.HUMAN:
                        self._notification = RobotNotification.FOUND_HUMAN
                        self._humans.append((floor, new_x, new_y))
                        self._map[floor, new_x, new_y] = MapObject.EMPTY
                    elif self._map[floor, new_x, new_y] == MapObject.OBSTACLE:
                        self._notification = RobotNotification.FOUND_OBSTACLE
                        self._obstacles.append((floor, new_x, new_y))
                        self._map[floor, new_x, new_y] = MapObject.EMPTY

    def run(self):  # przesuniecie robota po udzieleniu zgody na jazde gdy nie ma przeszkody
        robot_state = self.get_status()[1]
        if robot_state == RobotStatus.RUN:  # while still not arrived
            if self._notification == RobotNotification.WANT_RUN:
                self._status = (self._status[0] + 1, RobotStatus.RUN)
                self._notification = RobotNotification.NONE
            if self._movement_iterations > 0:
                self._movement_iterations -= 1  # decrease iterations steps
            elif self._movement_iterations <= 0:
                self._notification = RobotNotification.ARRIVED  # if arrived notify main driver
                self._status = (self._status[0], RobotStatus.STOP)

        elif robot_state == RobotStatus.STOP:
            self.detect_obstacle()
            if self._notification == RobotNotification.NONE and self.get_status()[0] < len(self._path) - 1:
                self._notification = RobotNotification.WANT_RUN
                self._movement_iterations = random.randint(2, 6)
