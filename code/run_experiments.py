#!/usr/bin/python
import argparse
from instance_generator import *
from cbs import CBSSolver
from independent import IndependentSolver
from sequential import SequentialSolver
from visualize import Animation

NUM_OBS = 10
NUM_AGENTS = 10
WIDTH = 10
TIME = 60
SOLVER = "CBS"

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs test suite for cbs')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file (or leave empty to generate random instances)')
    parser.add_argument('--agents', type=int, default=NUM_AGENTS,
                        help='The number of agents, defaults to ' + str(NUM_AGENTS))
    parser.add_argument('--grid', type=int, default=WIDTH,
                        help='The width of the square grid, defaults to ' + str(WIDTH))
    parser.add_argument('--obs', type=int, default=NUM_OBS,
                        help='The number of obstacles on the grid, defaults to ' + str(NUM_OBS))
    parser.add_argument('--seed', type=int, default=None,
                        help='The random seed (or leave empty to use time as the seed)')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--time', type=int, default=TIME,
                        help='The time limit of the solver (seconds), defaults to ' + str(TIME))
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Sequential}), defaults to ' + str(SOLVER))

    args = parser.parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    if args.instance is not None:
        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(args.instance)
    else:
        print("***Generate an instance***")
        my_map = gen_random_map(args.grid, args.obs)
        starts, goals = gen_random_starts_and_goals(my_map, args.agents)
    print_mapf_instance(my_map, starts, goals)
    # save_MAPF_instance('test.txt', my_map, starts, goals)

    if args.solver == "CBS":
        print("***Run CBS***")
        cbs = CBSSolver(my_map, starts, goals)
        paths = cbs.find_solution(args.disjoint, args.time)
    elif args.solver == "Independent":
        print("***Run Independent***")
        solver = IndependentSolver(my_map, starts, goals)
        paths = solver.find_solution()
    elif args.solver == "Sequential":
        print("***Run Sequential***")
        solver = SequentialSolver(my_map, starts, goals)
        paths = solver.find_solution()
    else:
        raise RuntimeError("Unknown solver!")

    print("***Test paths on a simulation***")
    animation = Animation(my_map, starts, goals, paths)
    # animation.save("output.mp4", 1.0)
    animation.show()
