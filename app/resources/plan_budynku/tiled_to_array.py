import numpy as np
import json
import os
import copy
import pickle
import random
from src.consts import MapObject


def get_map_details():
    directory_name = input("Podaj nazwe folderu zawierajacego mapy: ")
    humans_amount = input("Podaj liczbe ludzi: ")
    obstacle_amount = input("Podaj liczbe przeszkod: ")
    humans_amount = int(humans_amount)
    obstacle_amount = int(obstacle_amount)
    return directory_name, humans_amount, obstacle_amount


def get_map_file_names(directory):
    file_names = [os.path.normcase(f) for f in os.listdir(directory)]  # (1) (2)
    file_names = list(filter(lambda x: x.endswith('.json'), file_names))
    return file_names


def get_output_map_filename():
    output_filename = input("Podaj nazwe pliku do zapisu: ")
    return output_filename


def tiled_to_array(json_map_file_name):
    results = []
    with open(json_map_file_name, 'r') as json_map:
        data = json.load(json_map)
        for layer in data['layers']:
            height = layer['height']
            width = layer['width']
            data = layer['data']
            map_2d = np.resize(data, (height, width))
            results.append(map_2d)
    return np.array(results, dtype=object)


def create_maps(maps_dir):
    directory_name, humans_amount, obstacle_amount = get_map_details()
    directory_name = os.path.join(maps_dir, directory_name)
    floor_map_files = get_map_file_names(directory_name)
    map = []
    for floor_file_name in floor_map_files:
        floor_array = tiled_to_array(os.path.join(directory_name, floor_file_name))
        map.append(floor_array)
    map = np.vstack(map)
    map_original = copy.deepcopy(map)
    map[map == 1] = MapObject.WALL
    map[map == 0] = MapObject.EMPTY
    map[map == 2] = MapObject.STEPS
    steps = []
    for floor in map:
        for x in np.argwhere(map == MapObject.STEPS):
            steps.append(x)
    for _, x, y in steps:
        for floor in map:
            floor[x, y] = MapObject.STEPS

    map_obstacles = copy.deepcopy(map)
    for floor in map_obstacles:
        humans_per_floor = random.randint(1, humans_amount)
        obstacles_per_floor = random.randint(1, obstacle_amount)
        while humans_per_floor > 0:
            x = random.randint(0, floor.shape[0] - 1)
            y = random.randint(0, floor.shape[1] - 1)
            if floor[x, y] == MapObject.EMPTY:
                floor[x, y] = MapObject.HUMAN
                humans_per_floor -= 1
        while obstacles_per_floor > 0:
            x = random.randint(0, floor.shape[0] - 1)
            y = random.randint(0, floor.shape[1] - 1)
            if floor[x, y] == MapObject.EMPTY:
                floor[x, y] = MapObject.OBSTACLE
                obstacles_per_floor -= 1

        humans_amount -= humans_per_floor
        obstacle_amount -= obstacles_per_floor

    output_file_name = get_output_map_filename()
    with open(output_file_name + '.bin', 'wb') as f:
        pickle.dump(map, f)
    with open(output_file_name + '_obstacles.bin', 'wb') as f:
        pickle.dump(map_obstacles, f)

    # with open('3dcsv.txt', 'w') as f:
    #     for a in map_original:
    #         np.savetxt(f, a, fmt='%2d')
    #         f.write('\n')
    #
    # map_original = np.loadtxt('3dcsv.txt')
    # map_original = map_original.reshape((3, map_original.shape[0] // 3, map_original.shape[1]))
    # print(map_original)
    return output_file_name + '.bin', output_file_name + '_obstacles.bin'


if __name__ == "__main__":
    create_maps('.')
