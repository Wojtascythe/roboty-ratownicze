from src.consts import MapObject
from enum import Enum
from src import temporary_map
import random
import pygame


class Color:
    def __init__(self, value):
        self.value = value


def random_color_generator():
    # RGB color
    return Color((random.randint(1, 254), random.randint(200, 254), random.randint(1, 254)))


class Colors(Enum):
    """
    Colors used while drawing map
    """
    BLUE = (0, 0, 255)
    RED = (255, 0, 0)
    YELLOW = (255, 255, 0)
    GREEN = (0, 255, 0)
    WHITE = (255, 255, 255)
    BLACK = (0, 0, 0)
    ORANGE = (255, 150, 0)


class MapGui(object):
    MAP_OBJECT_TO_COLOR = {
        MapObject.EMPTY: Colors.WHITE, MapObject.WALL: Colors.BLACK, MapObject.HUMAN: Colors.RED,
        MapObject.VISITED: Colors.BLUE, MapObject.OBSTACLE: Colors.ORANGE, MapObject.STEPS: Colors.YELLOW
    }

    def __init__(self, square_size=10):
        self._square_size = square_size
        self._map = None
        self._drawn_map_height = None
        self._drawn_map_width = None
        self._height = None
        self._width = None
        self._size = None
        self._screen = None
        self._others_colors = dict()

    def update(self, map):
        self._map = map
        self._drawn_map_height = map.shape[1]
        self._drawn_map_width = map.shape[2] * map.shape[0]
        self._height = (self._drawn_map_height + 1) * self._square_size
        self._width = self._drawn_map_width * self._square_size
        self._size = (self._width, self._height)
        self._screen = pygame.display.set_mode(self._size)

    def draw(self, wait_time=100):
        pygame.init()
        for p in range(self._map.shape[0]):
            for r in range(self._map.shape[1]):
                for c in range(self._map.shape[2]):
                    element = self._map[p, r, c]
                    if element in self.MAP_OBJECT_TO_COLOR:
                        color = self.MAP_OBJECT_TO_COLOR[element]
                    else:
                        # color = Colors.GREEN
                        color = self._others_colors.get(element, None)
                        if color is None:
                            color = random_color_generator()
                            self._others_colors[element] = color
                    pygame.draw.rect(self._screen, color.value,
                                     (c * self._square_size + p * 30 * self._square_size, (r + 1) * self._square_size,
                                      self._square_size, self._square_size))

        pygame.display.update()
        pygame.time.wait(wait_time)


if __name__ == '__main__':
    W = MapObject.WALL
    E = MapObject.EMPTY
    H = MapObject.HUMAN
    O = MapObject.OBSTACLE
    S = MapObject.STEPS
    R = 1
    m_real_map = temporary_map.temporary_map_2_floors
    gui = MapGui()
    gui.update(m_real_map)
    gui.draw()
