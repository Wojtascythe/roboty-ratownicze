from .consts import (
    MapObject,
    RobotNotification,
    RobotStatus
)
import logging

import numpy as np
import random
import networkx as nx
import numbers
from operator import itemgetter
from itertools import chain


def banker(processes, available, occupied, max_depth):
    steps = []
    needed_resources = []
    for process in processes:
        needed_resources.append([])
        for i in range(max_depth):
            if i < len(process):
                needed_resources[-1].append(process[i])
            else:
                needed_resources[-1].append(process[-1])
    finished = np.full(len(processes), False, dtype=bool)
    current_position = np.zeros(len(processes), dtype=int)
    moved = True
    while moved:
        moved = False
        for finished_idx in range(len(finished)):
            if not finished[finished_idx]:
                for resource_idx, resource in enumerate(
                        needed_resources[finished_idx][current_position[finished_idx]:]):
                    others_proc_resources = needed_resources[:finished_idx] + needed_resources[finished_idx + 1:]
                    if resource not in list(chain.from_iterable(others_proc_resources)):
                        result = all(elem in available for elem in
                                     needed_resources[finished_idx][current_position[finished_idx]:resource_idx + 1])
                        if result:
                            moved = True
                            current_position[finished_idx] = resource_idx + 1
                            if current_position[finished_idx] == max_depth:
                                finished[finished_idx] = True
    return np.all(finished), steps


class MainDriver(object):
    def __init__(self, map, robots):
        self._map = np.copy(map)
        self._empty_map = np.copy(map)
        self._robots = robots
        self._paths = {r.get_id(): r._path.copy() for r in robots}
        self._robots_status = {r.get_id(): (0, RobotStatus.STOP) for r in robots}
        self.callbacks = {
            RobotNotification.NONE: MainDriver._robot_notify_none_callback,
            RobotNotification.ARRIVED: self._robot_notify_arrived_callback,
            RobotNotification.FOUND_HUMAN: self._robot_notify_found_human_callback,
            RobotNotification.FOUND_OBSTACLE: self._robot_notify_found_obstacle_callback,
            RobotNotification.WANT_RUN: self._robot_notify_want_run_callback,
        }
        self._robot_return_step = {r.get_id(): None for r in robots}
        self._robot_area = {r.get_id(): [] for r in robots}
        self._map_graph = nx.Graph()
        self._init_graph()
        self._init_areas()
        for robot in self._robots:
            self._set_map_field(robot._initial_localization, robot.get_id())

    def _init_areas(self):
        map_closed_areas, closed_areas = self._close_areas()
        sorted_closed_areas = sorted(closed_areas, key=lambda x: len(x))
        robots_amount = len(self._robots)
        distances = np.zeros((robots_amount, len(sorted_closed_areas)))
        for num_r, robot in enumerate(distances):
            for num_ar, area in enumerate(sorted_closed_areas):
                try:
                    path = self._graph_connection(self._robots[num_r]._initial_localization, area[0])
                    distances[num_r, num_ar] = len(path)
                except nx.NetworkXNoPath:
                    distances[num_r, num_ar] = float('inf')
        used_areas = set()
        while len(used_areas) < len(sorted_closed_areas):
            for num_r, robot in enumerate(distances):
                closest = -1
                for c in sorted(range(len(robot)), key=lambda k: robot[k]):
                    if c not in used_areas and distances[num_r, c]:
                        closest = c
                        used_areas.add(closest)
                        break
                if closest != float('inf'):
                    self._robot_area[self._robots[num_r].get_id()].append(sorted_closed_areas[closest])
        # for i, area in enumerate(sorted_closed_areas):
        #     self._robot_area[self._robots[i % robots_amount].get_id()].append(area)

    def _init_graph(self):
        available_map_objects = [MapObject.EMPTY, MapObject.STEPS, MapObject.VISITED]
        for z, floor in enumerate(self._map):
            for x, row in enumerate(floor):
                for y, col in enumerate(row):
                    if self._map[z, x, y] in available_map_objects or \
                            (isinstance(self._map[z, x, y], numbers.Number) and self._map[z, x, y] > 0):
                        self._map_graph.add_node((z, x, y))
        for (z, x, y), _ in self._map_graph.nodes.items():
            if self._map_graph.has_node((z, x + 1, y)):
                self._map_graph.add_edge((z, x, y), (z, x + 1, y))
            if self._map_graph.has_node((z, x - 1, y)):
                self._map_graph.add_edge((z, x, y), (z, x - 1, y))
            if self._map_graph.has_node((z, x, y + 1)):
                self._map_graph.add_edge((z, x, y), (z, x, y + 1))
            if self._map_graph.has_node((z, x, y - 1)):
                self._map_graph.add_edge((z, x, y), (z, x, y - 1))
            if self._map[z, x, y] == MapObject.STEPS:
                if self._map_graph.has_node((z + 1, x, y)) and self._map[z + 1, x, y] == MapObject.STEPS:
                    self._map_graph.add_edge((z, x, y), (z + 1, x, y))
                if self._map_graph.has_node((z - 1, x, y)) and self._map[z - 1, x, y] == MapObject.STEPS:
                    self._map_graph.add_edge((z, x, y), (z - 1, x, y))

    def _robot_notify_none_callback(self, robot):
        logging.info(f'_robot_notify_none_callback, robot {robot.get_id()}')
        pass

    def _clear_path(self, path):
        result_path = [path[0]]
        for p in path:
            if p != result_path[-1]:
                result_path.append(p)
        return result_path

    def _group_empty(self, empty):
        result = []

        def _group_empty_helper(position, area):
            z, x, y = position
            if position not in empty:
                return
            empty.discard(position)
            area.append((z, x, y))
            _group_empty_helper((z, x + 1, y), area)
            _group_empty_helper((z, x - 1, y), area)
            _group_empty_helper((z, x, y + 1), area)
            _group_empty_helper((z, x, y - 1), area)

        while len(empty) > 0:
            result.append([])
            _group_empty_helper(empty.pop(), result[-1])
        return result

    def _find_closest_not_explored(self, robot):
        source = self._get_robot_coordinates(robot)
        robots_left_max = float(0)
        for r in self._robots:
            robots_left_max = max(robots_left_max, self._robot_return_step[r.get_id()] - self._get_robot_step(r))
        paths = nx.single_source_dijkstra(self._map_graph, source, cutoff=robots_left_max)
        paths_length = []
        for k, v in paths[0].items():
            paths_length.append((v, paths[1][k]))
        not_visited = list(
            filter(lambda x: self._get_map_field(x[1][-1]) == MapObject.EMPTY and x[0] > 0, paths_length))
        paths_length = sorted(not_visited, key=itemgetter(0))
        return paths_length

    def _divide_area_again_plan(self, robot, closest_not_explored):
        if len(closest_not_explored) == 0:
            return
        robot_area = []
        robot_id = -1
        for id, areas in self._robot_area.items():
            for area in areas:
                if closest_not_explored[0][1][-1] in area:
                    robot_area = area
                    robot_id = id
                    break
        import matplotlib.pyplot as plt
        path, tmp_map = self._flood_fill(closest_not_explored[0][1][-1], robot_area)
        tmp_map[tmp_map == MapObject.EMPTY] = -10
        tmp_map[tmp_map == MapObject.WALL] = -20
        tmp_map[tmp_map == MapObject.VISITED] = -5
        tmp_map[tmp_map == MapObject.OBSTACLE] = -20
        tmp_map[tmp_map == MapObject.HUMAN] = -20
        tmp_map[tmp_map == MapObject.STEPS] = -1
        # plt.imshow(tmp_map[0].astype(np.float))
        # plt.show()
        # plt.imshow(tmp_map[1].astype(np.float))
        # plt.show()
        if len(path) > 2:
            self._robots_status[robot.get_id()] = (0, RobotStatus.STOP)
            self._paths[robot.get_id()] = closest_not_explored[0][1].copy()
            self._robot_area[robot.get_id()].append([])
            for i in range(len(path) // 2 + 1):
                try:
                    self._paths[robot_id].remove(path[i])
                except ValueError:
                    pass
                robot_area.remove(path[i])
                self._paths[robot.get_id()].append(path[i])
                self._robot_area[robot.get_id()][-1].append(path[i])
            self._robot_return_step[robot.get_id()] = len(self._paths[robot.get_id()])
            self.again_plan_path(robot=robot)
            robot.set_path(self._paths[robot.get_id()])
            robot.set_step(self._robots_status[robot.get_id()][0])
            other_robot = list(filter(lambda x: x.get_id() == robot_id, self._robots))[0]

            self.again_plan_path(robot=other_robot)
            other_robot.set_path(self._paths[other_robot.get_id()])
            other_robot.set_step(self._robots_status[other_robot.get_id()][0])

    def _robot_notify_arrived_callback(self, robot):
        logging.info(f'_robot_notify_arrived_callback, robot {robot.get_id()}')
        self._set_robot_status(robot, RobotStatus.STOP)
        robot_step = self._get_robot_step(robot)
        # if robot_step <= 0:
        #     return
        prev_coords = self._get_robot_coordinates(robot, -1)
        pz, px, py = prev_coords
        if type(self._get_map_field(prev_coords)) != MapObject \
                and self._empty_map[pz, px, py] == MapObject.EMPTY:
            self._set_map_field(prev_coords, MapObject.VISITED)
        else:
            self._set_map_field(prev_coords, self._empty_map[pz, px, py])
        floor, x, y = prev_coords
        floors, x_max, y_max = self._map.shape
        for x_i in range(3):
            for y_i in range(3):
                new_x = x - 1 + x_i
                new_y = y - 1 + y_i
                if 0 <= new_x < x_max and 0 <= new_y < y_max:
                    if self._map[floor, new_x, new_y] == MapObject.EMPTY:
                        self._map[floor, new_x, new_y] = MapObject.VISITED
        curr_coords = self._get_robot_coordinates(robot)
        self._set_map_field(curr_coords, robot.get_id())
        if robot_step == self._robot_return_step[robot.get_id()]:
            closest_not_explored = self._find_closest_not_explored(robot=robot)
            self._divide_area_again_plan(robot, closest_not_explored)

    def _robot_notify_found_human_callback(self, robot):
        logging.info(f'_robot_notify_found_human_callback, robot {robot.get_id()}')
        self._set_robot_status(robot, RobotStatus.STOP)
        obstacles = robot.read_and_clear_obstacles()
        humans = robot.read_and_clear_humans()
        for coords in obstacles:
            self._set_map_field(coords, MapObject.OBSTACLE)
        for coords in humans:
            self._set_map_field(coords, MapObject.HUMAN)
        self._map_graph.remove_nodes_from(humans)
        self._map_graph.remove_nodes_from(obstacles)

    def _robot_notify_found_obstacle_callback(self, robot):
        logging.info(f'_robot_notify_found_obstacle_callback, robot {robot.get_id()}')
        self._set_robot_status(robot, RobotStatus.STOP)
        obstacles = robot.read_and_clear_obstacles()
        humans = robot.read_and_clear_humans()
        logging.info(f'obstacles {obstacles}')
        logging.info(f'humans {humans}')
        for coords in obstacles:
            self._set_map_field(coords, MapObject.OBSTACLE)
        for coords in humans:
            self._set_map_field(coords, MapObject.HUMAN)
        self._map_graph.remove_nodes_from(humans)
        self._map_graph.remove_nodes_from(obstacles)

    def _robot_notify_want_run_callback(self, robot):
        def _find_non_blocking_config(path_1, path_2):
            path1 = []
            path2 = []
            if len(path_1) > 1 and len(path_2) > 1:
                if path_1[0] == path_2[1] and path_1[1] == path_2[0]:
                    current_pos = path_1[0]
                    i = 1
                    while i < len(path_2) and current_pos == path_2[i]:
                        z, x, y = current_pos
                        available = [(z, x + 1, y), (z, x - 1, y), (z, x, y + 1), (z, x, y - 1)]
                        for cord in available:
                            if self._get_map_field(cord) in [MapObject.VISITED, MapObject.STEPS, MapObject.EMPTY]:
                                current_pos = cord
                                path1.append(cord)
                                break
                        i += 1
            else:
                if len(path_1) == 1:
                    current_pos = path_1[0]
                    i = 1
                    while i < len(path_2) and current_pos == path_2[i]:
                        z, x, y = current_pos
                        available = [(z, x + 1, y), (z, x - 1, y), (z, x, y + 1), (z, x, y - 1)]
                        for cord in available:
                            if self._get_map_field(cord) in [MapObject.VISITED, MapObject.STEPS, MapObject.EMPTY]:
                                current_pos = cord
                                path1.append(cord)
                                break
                        i += 1
                elif len(path_2) == 1:
                    current_pos = path_2[0]
                    i = 1
                    while i < len(path_1) and current_pos == path_1[i]:
                        z, x, y = current_pos
                        available = [(z, x + 1, y), (z, x - 1, y), (z, x, y + 1), (z, x, y - 1)]
                        for cord in available:
                            if self._get_map_field(cord) in [MapObject.VISITED, MapObject.STEPS, MapObject.EMPTY]:
                                current_pos = cord
                                path2.append(cord)
                                break
                        i += 1
            print(path1, path2)
            return path1, path2

        logging.info(f'_robot_notify_want_run_callback, robot {robot.get_id()}')
        next_coordinates = self._get_robot_coordinates(robot, 1)
        next_coordinates_state = self._get_map_field(next_coordinates)
        if type(next_coordinates_state) != MapObject:
            opposite_robot_id = next_coordinates_state
            opposite_robot_path = self._paths[opposite_robot_id]
            opposite_robot_status = self._robots_status[opposite_robot_id]
            path1, path2 = _find_non_blocking_config(
                self._paths[robot.get_id()][self._robots_status[robot.get_id()][0]:],
                opposite_robot_path[opposite_robot_status[0]:])

            path1 = path1 + list(reversed(path1[:-1]))
            path2 = path2 + list(reversed(path2[:-1]))

            self._paths[robot.get_id()] = \
                self._paths[robot.get_id()][:self._robots_status[robot.get_id()][0] + 1] + \
                path1 + self._paths[robot.get_id()][self._robots_status[robot.get_id()][0] + 1:]
            self._robot_return_step[robot.get_id()] += len(path1)
            robot.set_path(self._paths[robot.get_id()])

            self._paths[opposite_robot_id] = opposite_robot_path[:opposite_robot_status[0] + 1] + \
                                             path2 + opposite_robot_path[opposite_robot_status[0] + 1:]
            self._robot_return_step[opposite_robot_id] += len(path2)
            list(filter(lambda x: x.get_id() == opposite_robot_id, self._robots))[0].set_path(
                self._paths[opposite_robot_id])
            return
        self._set_map_field(next_coordinates, robot.get_id())
        self._set_robot_status(robot, RobotStatus.RUN)
        self._next_robot_step(robot)

    def _get_robot_status(self, robot):
        return self._robots_status[robot.get_id()][1]

    def _set_robot_status(self, robot, status):
        self._robots_status[robot.get_id()] = (self._robots_status[robot.get_id()][0], status)
        robot.set_status(status)

    def _next_robot_step(self, robot):
        step, status = self._robots_status[robot.get_id()]
        self._robots_status[robot.get_id()] = (step + 1, status)

    def _set_robot_step(self, robot, step):
        _, status = self._robots_status[robot.get_id()]
        self._robots_status[robot.get_id()] = (step, status)

    def _get_robot_step(self, robot):
        return self._robots_status[robot.get_id()][0]

    def _get_robot_coordinates(self, robot, offset_from_current=0):
        return self._paths[robot.get_id()][self._get_robot_step(robot) + offset_from_current]

    def _get_map_field(self, coordinates):
        z, x, y = coordinates
        return self._map[z, x, y]

    def _set_map_field(self, coordinates, status):
        z, x, y = coordinates
        self._map[z, x, y] = status

    @staticmethod
    def _get_map_available_next_coords(current_coords, map):
        available_coords = []
        non_visited_coords = []
        z_c, x_c, y_c = current_coords
        if MapObject.STEPS == map[z_c, x_c, y_c]:
            for z in range(z_c - 1, z_c + 2, 2):
                if 0 <= z < map.shape[0]:
                    available_coords.append((z, x_c, y_c))
                    if map[z, x_c, y_c] != MapObject.VISITED:
                        non_visited_coords.append((z, x_c, y_c))
        for x in range(x_c - 1, x_c + 2, 2):
            if 0 <= x < map.shape[1] and (map[z_c, x, y_c] == MapObject.EMPTY
                                          or map[z_c, x, y_c] == MapObject.VISITED
                                          or map[z_c, x, y_c] == MapObject.STEPS):
                available_coords.append((z_c, x, y_c))
                if map[z_c, x, y_c] != MapObject.VISITED:
                    non_visited_coords.append((z_c, x, y_c))
        for y in range(y_c - 1, y_c + 2, 2):
            if 0 <= y < map.shape[2] and (map[z_c, x_c, y] == MapObject.EMPTY
                                          or map[z_c, x_c, y] == MapObject.VISITED
                                          or map[z_c, x_c, y] == MapObject.STEPS):
                available_coords.append((z_c, x_c, y))
                if map[z_c, x_c, y] != MapObject.VISITED:
                    non_visited_coords.append((z_c, x_c, y))

        return non_visited_coords if len(non_visited_coords) > 0 else available_coords

    def get_map(self):
        return self._map

    def _graph_connection(self, start_pos, dest_pos):
        return nx.dijkstra_path(self._map_graph, start_pos, dest_pos)

    def _flood_fill(self, start_pos, robot_area):
        tmp_map = np.copy(self._map)
        z, x, y = start_pos
        tmp_map[z, x, y] = MapObject.EMPTY
        z_size, x_size, y_size = self._map.shape
        path = []

        def flood_fill_helper(start_pos, i):
            z, x, y = start_pos
            if not (0 <= x < x_size and 0 <= y < y_size) or tmp_map[z, x, y] != MapObject.EMPTY:
                return
            if not (z, x, y) in robot_area:
                return
            i = i + 1
            tmp_map[z, x, y] = i
            path.append((z, x, y))
            flood_fill_helper((z, x + 1, y), i)
            flood_fill_helper((z, x - 1, y), i)
            flood_fill_helper((z, x, y + 1), i)
            flood_fill_helper((z, x, y - 1), i)

        flood_fill_helper(start_pos, 1)
        return path, tmp_map

    def _close_areas(self):
        result = np.copy(self._map)
        result_last = None
        z_size, x_size, y_size = result.shape
        W = MapObject.WALL
        E = MapObject.EMPTY
        mask1 = np.array([[E, W, E],
                          [E, E, E]])
        mask2 = np.array([[E, E, E],
                          [E, W, E]])
        mask3 = np.array([[E, E],
                          [W, E],
                          [E, E]])
        mask4 = np.array([[E, E],
                          [E, W],
                          [E, E]])
        while not np.array_equal(result_last, result):
            result_last = np.copy(result)
            for z in range(z_size):
                for x in range(x_size - 1):
                    for y in range(y_size - 2):
                        if np.all(result[z, x:x + 2, y:y + 3] == mask1):
                            result[z, x + 1, y + 1] = MapObject.WALL
                        if np.all(result[z, x:x + 2, y:y + 3] == mask2):
                            result[z, x, y + 1] = MapObject.WALL
                for x in range(x_size - 2):
                    for y in range(y_size - 1):
                        if np.all(result[z, x:x + 3, y:y + 2] == mask3):
                            result[z, x + 1, y + 1] = MapObject.WALL
                        if np.all(result[z, x:x + 3, y:y + 2] == mask4):
                            result[z, x + 1, y] = MapObject.WALL
        empty = set([tuple(x.reshape(1, -1)[0]) for x in np.argwhere(result == MapObject.EMPTY)])
        diff_walls = set([tuple(x.reshape(1, -1)[0]) for x in np.argwhere(self._map == MapObject.EMPTY)]) - empty
        closed = []
        while len(empty) > 0:
            new_element = empty.pop()
            closed.append([])
            neighbours = [new_element]
            while len(neighbours) > 0:
                z_e, x_e, y_e = neighbours.pop(0)
                closed[-1].append((z_e, x_e, y_e))
                for x in range(x_e - 1, x_e + 2):
                    for y in range(y_e - 1, y_e + 2):
                        if (z_e, x, y) in empty:
                            neighbours.append((z_e, x, y))
                        if (z_e, x, y) in diff_walls:
                            closed[-1].append((z_e, x, y))
                        empty.discard((z_e, x, y))
                        diff_walls.discard((z_e, x, y))
        return result, closed

    def plan_path(self, **kwargs):
        robot = kwargs['robot']
        id = robot.get_id()
        robot_areas = self._robot_area[id]
        current_robot_step, current_robot_status = self._robots_status[id]

        robot_coords_list = self._paths[id]  # get robot path list
        robot_position_plan_path = robot_coords_list[current_robot_step]  # get current robot position
        if current_robot_status == RobotStatus.STOP:
            self._paths[id] = [robot_position_plan_path]  # reset path list
            self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter
        else:
            self._robots_status[id] = (1, RobotStatus.RUN)
            self._paths[id] = [robot_coords_list[current_robot_step - 1], robot_position_plan_path]

        result_path = []
        available_areas = []
        for area_idx, area in enumerate(robot_areas):
            try:
                path_to_area = self._graph_connection(robot_position_plan_path, area[0])
                area_init_position = next(x for x in path_to_area if x in set(area))
                path_to_area = path_to_area[:path_to_area.index(area_init_position)]
                result_path = result_path + path_to_area
                robot_position_plan_path = area_init_position
                path, _ = self._flood_fill(robot_position_plan_path, area)
                for i in range(len(path) - 1):
                    if abs(path[i][1] - path[i + 1][1]) + abs(path[i][2] - path[i + 1][2]) > 1:
                        connect_path = self._graph_connection(path[i], path[i + 1])
                        result_path = result_path + connect_path
                    else:
                        result_path.append(path[i])
                robot_position_plan_path = result_path[-1]
                available_areas.append(area)
            except nx.NetworkXNoPath:
                logging.error('Cannot find path to area')
        self._robot_area[id] = available_areas
        try:
            return_path = self._graph_connection(robot_position_plan_path, robot._initial_localization)
        except nx.NetworkXNoPath:
            logging.error('Cannot find return path')
            return_path = []
        result_path = result_path + return_path
        self._paths[id] = self._clear_path(result_path)
        self._robot_return_step[id] = len(self._paths[id]) - len(return_path)
        # print(self._robot_return_step[id])
        return self._paths[id]

    def again_plan_path(self, **kwargs):
        robot = kwargs['robot']
        id = robot.get_id()
        current_robot_step, current_robot_status = self._robots_status[id]
        robot_coords_list = self._paths[id]  # get robot path list
        robot_position_plan_path = robot_coords_list[current_robot_step]  # get current robot position
        init_rp = robot_position_plan_path

        if current_robot_status == RobotStatus.STOP:
            path = [robot_position_plan_path]
            self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter
        else:
            self._robots_status[id] = (1, RobotStatus.RUN)
            path = [robot_coords_list[current_robot_step - 1], robot_position_plan_path]

        result_path = [robot_position_plan_path]

        left_steps = robot_coords_list[current_robot_step:self._robot_return_step[id] + 1]
        for i, (z, x, y) in enumerate(left_steps):
            if self._map[z, x, y] == MapObject.EMPTY:
                path.append((z, x, y))
                # robot_position_plan_path = (z, x, y)

        for i in range(len(path)):
            if abs(robot_position_plan_path[0] - path[i][0]) + \
                    abs(robot_position_plan_path[1] - path[i][1]) + \
                    abs(robot_position_plan_path[2] - path[i][2]) > 1:
                try:
                    connect_path = self._graph_connection(robot_position_plan_path,
                                                          path[i])
                    result_path = result_path + connect_path
                except nx.NetworkXNoPath:
                    pass
                except nx.NodeNotFound:
                    pass
            else:
                result_path.append(path[i])
            robot_position_plan_path = result_path[-1]
        try:
            return_path = self._graph_connection(robot_position_plan_path, robot._initial_localization)
        except nx.NetworkXNoPath:
            logging.error('Cannot find return path')
            return_path = []
        result_path = result_path + return_path
        self._paths[id] = self._clear_path(result_path)
        self._robot_return_step[id] = len(self._paths[id]) - len(return_path)
        # print(self._robot_return_step[id])
        logging.info(f'{init_rp}, {self._paths[id][0]}')
        return self._paths[id]

    def again_plan_paths(self, **kwargs):
        for robot in self._robots:
            self.again_plan_path(robot=robot)

    def plan_paths(self, **kwargs):
        for robot in self._robots:
            self.plan_path(robot=robot)

    def plan_random_path(self, **kwargs):
        path_length = kwargs['length']
        robot = kwargs['robot']
        id = robot.get_id()
        current_robot_step, current_robot_status = self._robots_status[id]

        robot_coords_list = self._paths[id]  # get robot path list
        robot_position_plan_path = robot_coords_list[current_robot_step]  # get current robot position
        if current_robot_status == RobotStatus.STOP:
            self._paths[id] = [robot_position_plan_path]  # reset path list
            self._robots_status[id] = (0, RobotStatus.STOP)  # reset steps counter
        else:
            self._robots_status[id] = (1, RobotStatus.RUN)
            self._paths[id] = [robot_coords_list[current_robot_step - 1], robot_position_plan_path]
        for step in range(path_length):  # create path starting from current position
            available_coords = set(MainDriver._get_map_available_next_coords(robot_position_plan_path, self._map))
            robot_position_plan_path_next = random.choice(list(available_coords))
            while robot_position_plan_path_next in self._paths[id] and len(available_coords) > 1:
                available_coords.remove(robot_position_plan_path_next)
                robot_position_plan_path_next = random.choice(list(available_coords))
            robot_position_plan_path = robot_position_plan_path_next
            self._paths[id].append(robot_position_plan_path)
        return self._paths[id]

    def plan_random_paths(self):
        for robot in self._robots:
            self.plan_random_path(robot=robot, length=1000)

    def send_paths_to_robots(self):
        for robot in self._robots:
            robot.set_path(self._paths[robot.get_id()])
            robot.set_step(self._robots_status[robot.get_id()][0])

    def handle_robot_notify(self, robot):
        robot_notification = robot.get_notify()
        if robot_notification == RobotNotification.NONE:
            self._robot_notify_none_callback(robot)
        elif robot_notification == RobotNotification.WANT_RUN:
            self._robot_notify_want_run_callback(robot)
        elif robot_notification == RobotNotification.ARRIVED:
            self._robot_notify_arrived_callback(robot)
        elif robot_notification == RobotNotification.FOUND_OBSTACLE:
            self._robot_notify_found_obstacle_callback(robot)
            self.again_plan_paths()
            self.send_paths_to_robots()
        elif robot_notification == RobotNotification.FOUND_HUMAN:
            self._robot_notify_found_human_callback(robot)
            self.again_plan_paths()
            self.send_paths_to_robots()
        else:
            raise Exception("Illegal notification")
