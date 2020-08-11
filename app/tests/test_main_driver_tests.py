import pytest
import numpy as np
from collections import Counter
from src.main_driver import MainDriver
from src.consts import MapObject, RobotNotification, RobotStatus


@pytest.fixture()
def map_empty():
    w = MapObject.WALL
    e = MapObject.EMPTY
    floor = [[e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e],
             [e, e, e, e, e, e, e]]
    return np.array([floor])


@pytest.fixture()
def map_with_walls():
    w = MapObject.WALL
    e = MapObject.EMPTY
    floor = [[w, w, w, w, w, w, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w],
             [w, e, e, e, e, e, w]]
    return np.array([floor])


@pytest.fixture()
def robots(mocker):
    robot = mocker.MagicMock()
    robot.get_id.return_value = 1
    return [robot]


def test_get_available_coordinates_in_empty_map(map_empty):
    coords = MainDriver._get_map_available_next_coords((0, 0, 0), map_empty)
    c1 = Counter(coords)
    c2 = Counter([(0, 1, 0), (0, 0, 1)])
    assert c1 == c2

    coords = MainDriver._get_map_available_next_coords((0, map_empty.shape[1] - 1, map_empty.shape[2] - 1), map_empty)
    c1 = Counter(coords)
    c2 = Counter([
        (0, map_empty.shape[1] - 2, map_empty.shape[2] - 1),
        (0, map_empty.shape[1] - 1, map_empty.shape[2] - 2)])
    assert c1 == c2


def test_get_available_coordinates_with_walls(map_with_walls):
    coords = MainDriver._get_map_available_next_coords((0, 1, 1), map_with_walls)
    c1 = Counter(coords)
    c2 = Counter([(0, 1, 2), (0, 2, 1)])
    assert c1 == c2

    coords = MainDriver._get_map_available_next_coords(
        (0, map_with_walls.shape[1] - 1, map_with_walls.shape[2] - 2), map_with_walls
    )
    c1 = Counter(coords)
    c2 = Counter([
        (0, map_with_walls.shape[1] - 2, map_with_walls.shape[2] - 2),
        (0, map_with_walls.shape[1] - 1, map_with_walls.shape[2] - 3)])
    assert c1 == c2


def test_plan_random_path(map_with_walls, robots):
    main_driver = MainDriver(map_with_walls, robots)
    new_path = main_driver.plan_random_path(robot=robots[0], length=10)
    assert 11 == len(new_path)  # current position+10 next steps


def test_handle_robot_notify_none(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.NONE
    mocker.spy(main_driver, '_robot_notify_none_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_none_callback.call_count == 1


def test_handle_robot_notify_arrived(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.ARRIVED
    robot = robots[0]
    main_driver._paths[robot.get_id()].append((0, 2, 2))
    main_driver._paths[robot.get_id()].append((0, 2, 3))
    main_driver._next_robot_step(robot)
    main_driver._next_robot_step(robot)
    main_driver._set_robot_status(robot, RobotStatus.RUN)
    mocker.spy(main_driver, '_robot_notify_arrived_callback')
    assert RobotStatus.RUN == main_driver._get_robot_status(robot)
    main_driver.handle_robot_notify(robot)
    assert main_driver._robot_notify_arrived_callback.call_count == 1
    assert RobotStatus.STOP == main_driver._get_robot_status(robot)
    previous_field = main_driver._get_robot_coordinates(robot, -1)
    assert MapObject.VISITED == main_driver._get_map_field(previous_field)
    assert 0 < main_driver._get_map_field(main_driver._get_robot_coordinates(robot))


def test_get_robot_status(map_with_walls, robots):
    main_driver = MainDriver(map_with_walls, robots)
    assert RobotStatus.STOP == main_driver._get_robot_status(robots[0])


def test_set_robot_status(map_with_walls, robots):
    main_driver = MainDriver(map_with_walls, robots)
    main_driver._set_robot_status(robots[0], RobotStatus.RUN)
    assert RobotStatus.RUN == main_driver._get_robot_status(robots[0])


def test_robot_next_step(map_with_walls, robots):
    main_driver = MainDriver(map_with_walls, robots)
    prev_step = main_driver._get_robot_step(robots[0])
    main_driver._next_robot_step(robots[0])
    current_step = main_driver._get_robot_step(robots[0])
    assert current_step == prev_step + 1


def test_handle_robot_notify_want_run(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    main_driver._paths[robots[0].get_id()].append((0, 1, 1))
    main_driver._paths[robots[0].get_id()].append((0, 1, 2))
    main_driver._paths[robots[0].get_id()].append((0, 1, 3))
    main_driver._set_robot_step(robots[0], 0)
    robots[0].get_notify.return_value = RobotNotification.WANT_RUN
    mocker.spy(main_driver, '_robot_notify_want_run_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_want_run_callback.call_count == 1


def test_handle_robot_notify_want_run_next_step_available(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.WANT_RUN
    robot = robots[0]
    main_driver._paths[robot.get_id()].append((0, 1, 1))
    main_driver._paths[robot.get_id()].append((0, 1, 2))
    main_driver._paths[robot.get_id()].append((0, 1, 3))
    main_driver._set_robot_status(robot, RobotStatus.STOP)
    main_driver._set_robot_step(robot, 0)
    main_driver._set_map_field((0, 1, 1), robot.get_id())
    mocker.spy(main_driver, '_robot_notify_want_run_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_want_run_callback.call_count == 1
    assert 0 < main_driver._get_map_field(main_driver._get_robot_coordinates(robot))
    assert main_driver._get_map_field(main_driver._get_robot_coordinates(robot, -1)) == \
           main_driver._get_map_field(main_driver._get_robot_coordinates(robot))


def test_handle_robot_notify_found_human(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.FOUND_HUMAN
    robots[0].get_position.return_value = (0, 2, 2)
    mocker.spy(main_driver, '_robot_notify_found_human_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_found_human_callback.call_count == 1


def test_handle_robot_notify_found_obstacle(map_with_walls, robots, mocker):
    main_driver = MainDriver(map_with_walls, robots)
    robots[0].get_notify.return_value = RobotNotification.FOUND_OBSTACLE
    robots[0].get_position.return_value = (0, 2, 2)
    mocker.spy(main_driver, '_robot_notify_found_obstacle_callback')
    main_driver.handle_robot_notify(robots[0])
    assert main_driver._robot_notify_found_obstacle_callback.call_count == 1
