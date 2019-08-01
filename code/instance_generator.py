#!/usr/bin/python
# This file is purely intended to generate MAPF instances
import random
from collections import deque
from pathlib import Path

NUM_WALKS = 10000


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def is_connected(my_map, start, goal):
    open_list = deque()
    closed_list = set()
    open_list.append(start)
    closed_list.add(start)
    while len(open_list) > 0:
        curr = open_list.popleft()
        if curr == goal:
            return True
        for i in range(4):
            next = move(curr, i)
            if my_map[next[0]][next[1]] or next in closed_list:
                continue
            open_list.append(next)
            closed_list.add(next)
    return False


def gen_random_map(size, num_of_obs):
    """Generates a random, fully-connected, square world of size cells per side.
    size        - cells per sided
    num_of_obs  - number of obstacle
    returns     - my_map
    """

    if num_of_obs > size * size or num_of_obs < 0:
        raise ValueError('Improper obstacle obs_density')

    size += 2  # two additional cols and rows for obstacle fences
    my_map = [[False for _ in range(size)] for _ in range(size)]

    # add obstacle fences
    for i in range(size):
        my_map[0][i] = True
        my_map[i][0] = True
        my_map[-1][i] = True
        my_map[i][-1] = True

    i = 0
    while i < num_of_obs:
        obs = (random.randint(0, size - 1), random.randint(0, size - 1))
        if my_map[obs[0]][obs[1]]:
            continue  # this location already has an obstacle
        my_map[obs[0]][obs[1]] = True
        dir1 = 0
        dir2 = 1
        while dir1 < 3 and dir2 < 4:
            start = move(obs, dir1)
            goal = move(obs, dir2)
            if my_map[start[0]][start[1]]:
                dir1 += 1
            elif dir2 <= dir1:
                dir2 = dir1 + 1
            elif my_map[goal[0]][goal[1]]:
                dir2 += 1
            elif is_connected(my_map, start, goal):
                dir1 = dir2
                dir2 += 1
            else:
                my_map[obs[0]][obs[1]] = False
                i -= 1
                break
        i += 1

    return my_map


def gen_random_starts_and_goals(my_map, num_of_agents, random_walk_steps=NUM_WALKS):
    """Generate random start and goal locations by random walk.
    my_map          - binary obstacle maps
    num_of_agents   - number of agents
    """

    size_x = len(my_map)
    size_y = len(my_map[1])
    # Generate the initial positions of the robots
    starts = []
    goals = []
    used4starts = [[False for _ in range(size_y)] for _ in range(size_x)]
    used4goals = [[False for _ in range(size_y)] for _ in range(size_x)]
    while len(starts) < num_of_agents:
        # Generate possible initial and goal positions
        start = (random.randint(0, size_x - 1), random.randint(0, size_y - 1))
        if my_map[start[0]][start[1]] or used4starts[start[0]][start[1]]:
            continue
        curr = start
        i = 0
        while i < random_walk_steps or used4goals[curr[0]][curr[1]]:
            r = random.randint(0, 3)
            next = move(curr, r)
            if my_map[next[0]][next[1]] is False:
                curr = next
                i += 1
        goal = curr
        starts.append(start)
        used4starts[start[0]][start[1]] = True
        goals.append(goal)
        used4goals[goal[0]][goal[1]] = True

    return starts, goals


def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def save_mapf_instance(filename, my_map, starts, goals):
    f = open(filename, 'w')
    f.write('map\n')
    for row in my_map:
        for cell in row:
            if cell:
                f.write('@ ')
            else:
                f.write('. ')
        f.write('\n')
    f.write('start locations:\n')
    for start in starts:
        f.write("{},{}\n".format(start[0], start[1]))
    f.write('goal locations:\n')
    for goal in goals:
        f.write("{},{}\n".format(goal[0], goal[1]))
    f.close()


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    line = f.readline()
    assert line == 'map\n'
    my_map = []
    line = f.readline()
    while line != 'start locations:\n':
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
        line = f.readline()

    starts = []
    line = f.readline()
    while line != 'goal locations:\n':
        line = line[:-1].split(',')
        assert (len(line) == 2)
        starts.append((int(line[0]), int(line[1])))
        line = f.readline()

    goals = []
    for line in f.readlines():
        line = line[:-1].split(',')
        assert (len(line) == 2)
        goals.append((int(line[0]), int(line[1])))

    f.close()

    assert (len(starts) == len(goals))

    return my_map, starts, goals
