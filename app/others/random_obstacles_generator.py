from random import random
from src.consts import MapObject
import src.gui as gui
import numpy as np
import src.temporary_map as tmp_map


def random_obstacles_generator(empty_map, obstacle_prob, human_prob, avoid_areas=None):
    if avoid_areas is None:
        avoid_areas = set()
    tmp_avoid_areas = set()
    for i in avoid_areas:
        z, x, y = i
        for xi in range(-1, 2):
            for yi in range(-1, 2):
                tmp_avoid_areas.add((z, x + xi, y + yi))
    avoid_areas = tmp_avoid_areas
    result_map = np.zeros_like(empty_map)
    for f, floor in enumerate(empty_map):
        for r, row in enumerate(floor):
            for c, col in enumerate(row):
                prob = random()
                if empty_map[f, r, c] == MapObject.EMPTY and \
                        (prob < obstacle_prob or prob < human_prob) and not (f, r, c) in avoid_areas:
                    if obstacle_prob < human_prob:
                        if prob < obstacle_prob:
                            result_map[f, r, c] = MapObject.OBSTACLE
                        elif prob < human_prob:
                            result_map[f, r, c] = MapObject.HUMAN
                    else:
                        if prob < human_prob:
                            result_map[f, r, c] = MapObject.HUMAN
                        elif prob < obstacle_prob:
                            result_map[f, r, c] = MapObject.OBSTACLE
                else:
                    result_map[f, r, c] = empty_map[f, r, c]
    return result_map


if __name__ == '__main__':
    m_gui = gui.MapGui()
    m_gui.update(tmp_map.temporary_map_1_floors)
    m_gui.draw(1000)
    m_gui.update(random_obstacles_generator(tmp_map.temporary_map_1_floors, 0.10, 0.05))
    m_gui.draw(1000)
