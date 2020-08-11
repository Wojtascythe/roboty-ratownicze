from enum import Enum


class RobotStatus(Enum):
    """
        Flaga wysyłana od sterownika nadrzędnego do sterownika robota
    """
    STOP = 1
    RUN = 2


class RobotNotification(Enum):
    """
        Informacja od robota przesyłana do sterownika nadrzędnego
    """
    NONE = 0
    ARRIVED = 1
    FOUND_HUMAN = 2
    FOUND_OBSTACLE = 3
    WANT_RUN = 4


class MapObject(Enum):
    """
        Objekty przechowywane w mapie
        Lokalizacja robotów oznaczana poprzez indeks robota (wartość dodatnia)
    """
    EMPTY = -1
    WALL = -2
    OBSTACLE = -3
    HUMAN = -4
    VISITED = -5
    STEPS = -6
