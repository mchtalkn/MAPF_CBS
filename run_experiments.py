#!/usr/bin/python
import argparse
import glob
import random
import time
from datetime import datetime
from pathlib import Path

from xlwt import Workbook

from cbs_better_heuristic import cbs
from old_implementations.my_a_star_v_2_heapdict import *
from old_implementations.prioritized_search import prioritized_search
from old_implementations.single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from visualize import Animation

# from my_a_star_v_2_heapdict_g import my_a_star as my_a_star2
sheet_flag = False
wb = Workbook()
sheet1 = wb.add_sheet('Sheet 1')
sheet1.write(0, 0, 'file')
sheet1.write(0, 1, 'start')
sheet1.write(0, 2, 'goal')
sheet1.write(0, 3, 'total time')
sheet1.write(0, 4, 'cost')
sheet1.write(0, 5, 'number of collisions at node')
sheet1.write(0, 6, 'number of collisions at edge')
sheet1.write(0, 7, 'number of agents that no path found for')
SOLVER = "A*"


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


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif not cell.isspace():
                my_map[-1].append(False)
            # else:
            #    my_map[-1].append(False)
    # #agents
    line = f.readline()
    starts = []
    goals = []
    if args.random_agents == 0:
        num_agents = int(line)
        # #agents lines with the start/goal positions
        for a in range(num_agents):
            line = f.readline()
            sx, sy, gx, gy = [int(x) for x in line.split(' ')]
            starts.append((sx, sy))
            goals.append((gx, gy))
    else:
        i = 0
        while i < args.random_agents:
            start_x = random.randint(0, len(my_map) - 1)
            start_y = random.randint(0, len(my_map[0]) - 1)
            goal_x = random.randint(0, len(my_map) - 1)
            goal_y = random.randint(0, len(my_map[0]) - 1)
            while goal_x != start_x and goal_y != start_y:
                goal_x = random.randint(0, len(my_map) - 1)
                goal_y = random.randint(0, len(my_map[0]) - 1)
            if my_map[start_x][start_y] is False and my_map[goal_x][goal_y] is False:
                starts.append((start_x, start_y))
                goals.append((goal_x, goal_y))
                i += 1
    f.close()
    return my_map, starts, goals


def check_paths(paths):
    position_table = {}
    node_collisions = []
    edge_collisions = []
    edge_table = {}
    no_path_indexes = []
    for agent in range(len(paths)):
        path = paths[agent]
        if path is not None and path != []:
            for time in range(len(path)):
                if time == 0:
                    continue
                if position_table.get((path[time], time)) is None:
                    position_table[(path[time], time)] = [agent]
                else:
                    position_table[(path[time], time)].append(agent)
                if edge_table.get([min(path[time], path[time - 1]),
                                   max(path[time], path[time - 1]), time]) is None:
                    edge_table[(min(path[time], path[time - 1]),
                                max(path[time], path[time - 1]), time)] = [agent]
                else:
                    edge_table[(min(path[time], path[time - 1]),
                                max(path[time], path[time - 1]), time)].append(agent)
        else:
            no_path_indexes.append(agent)
    for key, value in position_table.items():
        if len(value) > 1:
            node_collisions.append((key, value))
    for key, value in edge_table.items():
        if len(value) > 1:
            edge_collisions.append((key, value))
    return no_path_indexes, node_collisions, edge_collisions


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))
    parser.add_argument('--random_agents', type=int, default=0,
                        help='use given number of agents with random locations ')
    parser.add_argument('--loop', type=int, default=1,
                        help='loop over instances for given time ' + str(SOLVER))
    args = parser.parse_args()

    row = 1
    result_file = open("results.csv", "w", buffering=1)
    print(sorted(glob.glob(args.instance)))
    #
    # file='instances/test_10.txt'
    # if file:
    for i in range(args.loop):
        for file in sorted(glob.glob(args.instance)):
            print(f"***Import an instance : {file}***")
            my_map, starts, goals = import_mapf_instance(file)
            print_mapf_instance(my_map, starts, goals)
            if args.solver == "A*":
                print("***Run Single Agent A* ***")
                heuristic = compute_heuristics(my_map, goals[0])
                # constraints = [(0,(2,2),1),(0,(3,1),1)]
                constraints = []
                paths = []
                path = a_star(my_map, starts[0], goals[0], heuristic, 0, constraints)
                print(path)
                paths.append(path)
            elif args.solver == "my_A*":
                heuristic = my_compute_heuristics(my_map, goals[0])
                constraints = []
                paths = []
                path = my_a_star(my_map, starts[0], goals[0], heuristic, 0, constraints)
                print(path)
                paths.append(path)
            elif args.solver == 'multi_A*':
                paths = []
                constraints = []
                position_table = {}
                node_collisions = []
                edge_collisions = []
                edge_table = {}
                for agent in range(len(starts)):
                    heuristic = my_compute_heuristics(my_map, goals[agent])
                    path = my_a_star(my_map, starts[agent], goals[agent], heuristic, agent, constraints)
                    paths.append(path)
                    print(f'agent {agent} from:{starts[agent]} to {goals[agent]} \
                    path: {path}')
            elif args.solver == "multi_collision":
                start_time = time.time()
                paths = []
                constraints = []
                edge_constraints = []
                for agent in range(len(starts)):
                    for path in paths:
                        if path != None:
                            for i in range(len(path)):
                                constraints.append((agent, path[i], i))
                                if i == 0:
                                    continue
                                else:
                                    edge_constraints.append((agent, (path[i], path[i - 1]), i))
                    heuristic = my_compute_heuristics(my_map, goals[agent])
                    path = my_a_star(my_map, starts[agent], goals[agent], heuristic, agent, constraints,
                                     edge_constraints)
                    paths.append(path)
                multi_collision_time = time.time() - start_time
                sheet1.write(row, 0, file)
                sheet1.write(row, 1, str(starts))
                sheet1.write(row, 2, str(goals))
                sheet1.write(row, 3, multi_collision_time)
            elif args.solver == "test_single":
                if not sheet_flag:
                    sheet1.write(0, 8, 'time1')
                    sheet1.write(0, 9, 'time2')
                    sheet1.write(0, 10, '(time1-time2)/time1')
                    sheet1.write(0, 11, 'heuristic computation time1')
                    sheet1.write(0, 12, 'heuristic computation time2')
                    sheet1.write(0, 13, '(htime1-htime2)/htime1')
                    sheet1.write(0, 14, 'cost1')
                    sheet1.write(0, 15, 'cost2')
                    sheet_flag = True
                print(file)
                constraints = []
                paths1 = []
                paths2 = []
                for i in range(len(starts)):
                    sheet1.write(row, 0, file)
                    sheet1.write(row, 1, str(starts[i]))
                    sheet1.write(row, 2, str(goals[i]))
                    paths1 = []
                    paths2 = []
                    print(f'ex:{i} from {starts[i]} to {goals[i]}')
                    start_time = time.time()
                    heuristic1 = my_compute_heuristics(my_map, goals[i])
                    htime1 = time.time() - start_time
                    start_time = time.time()
                    path1 = my_a_star(my_map, starts[i], goals[i], heuristic1, i, constraints)
                    time1 = time.time() - start_time
                    start_time = time.time()
                    heuristic2 = compute_heuristics(my_map, goals[i])
                    htime2 = time.time() - start_time
                    start_time = time.time()
                    path2 = a_star(my_map, starts[i], goals[i], heuristic2, i, constraints)
                    time2 = time.time() - start_time
                    paths1.append(path1)
                    paths2.append(path2)
                    cost1 = get_sum_of_cost(paths1)
                    cost2 = get_sum_of_cost(paths2)
                    sheet1.write(row, 8, time1)
                    sheet1.write(row, 9, time2)
                    sheet1.write(row, 10, (time1 - time2) / time1)
                    sheet1.write(row, 11, htime1)
                    sheet1.write(row, 12, htime2)
                    sheet1.write(row, 13, (htime1 - htime2) / htime1)
                    sheet1.write(row, 14, cost1)
                    sheet1.write(row, 15, cost2)
                    paths1.append(path1)
                    row += 1
                    if cost1 != cost2:
                        print(f'cost difference {cost1 - cost2}')
                    paths = paths1
            elif args.solver == "prioritized_search":
                start_time = time.time()
                paths = prioritized_search(my_map, starts, goals)
                prioritized_search_time = time.time() - start_time
                sheet1.write(row, 0, file)
                sheet1.write(row, 1, str(starts))
                sheet1.write(row, 2, str(goals))
                sheet1.write(row, 3, prioritized_search_time)
            elif args.solver == 'cbs':
                start_time = time.time()
                paths, _ = cbs(my_map, starts, goals)
                cbs_time = time.time() - start_time
                sheet1.write(row, 0, file)
                sheet1.write(row, 1, str(starts))
                sheet1.write(row, 2, str(goals))
                sheet1.write(row, 3, cbs_time)

            elif args.solver == 'multi_test':
                print('greedy')
                start_time = time.time()
                paths = []
                constraints = []
                edge_constraints = []
                for agent in range(len(starts)):
                    for path in paths:
                        if path != None:
                            for i in range(len(path)):
                                constraints.append((agent, path[i], i))
                                if i == 0:
                                    continue
                                else:
                                    edge_constraints.append((agent, (path[i], path[i - 1]), i))
                    heuristic = my_compute_heuristics(my_map, goals[agent])
                    path = my_a_star(my_map, starts[agent], goals[agent], heuristic, agent, constraints,
                                     edge_constraints)
                    paths.append(path)
                multi_collision_time = time.time() - start_time
                sheet1.write(row, 0, file)
                sheet1.write(row, 1, str(starts))
                sheet1.write(row, 2, str(goals))
                sheet1.write(row, 3, multi_collision_time)
                print('prioritized_search')
                start_time = time.time()
                paths2 = prioritized_search(my_map, starts, goals)
                prioritized_search_time = time.time() - start_time
                cost2 = get_sum_of_cost(paths2)
                no_path_indexes2, node_collisions2, edge_collisions2 = check_paths(paths2)
                sheet1.write(row, 8, prioritized_search_time)
                sheet1.write(row, 9, cost2)
                sheet1.write(row, 10, len(no_path_indexes2))
                print('cbs')
                start_time = time.time()
                paths3, _ = cbs(my_map, starts, goals)
                prioritized_search_time = time.time() - start_time
                cost3 = get_sum_of_cost(paths3)
                no_path_indexes3, node_collisions3, edge_collisions3 = check_paths(paths3)
                sheet1.write(row, 11, prioritized_search_time)
                sheet1.write(row, 12, cost3)
                sheet1.write(row, 13, len(no_path_indexes3))
            else:
                raise RuntimeError("Unknown solver!")

            cost = get_sum_of_cost(paths)
            result_file.write("{},{}\n".format(file, cost))
            no_path_indexes, node_collisions, edge_collisions = check_paths(paths)
            no_path = [(i, starts[i], goals[i]) for i in no_path_indexes]
            sheet1.write(row, 4, cost)
            sheet1.write(row, 5, len(node_collisions))
            sheet1.write(row, 6, len(edge_collisions))
            sheet1.write(row, 7, len(no_path))
            row += 1
            print(f"""file:{file} \n number of node collisions:{len(node_collisions)}\n
                node collisions:{node_collisions}\n number of edge collisions:
                {len(edge_collisions)} \n edge collisions: {edge_collisions}""")
            for agent, f, t in no_path:
                print(f'no path found for agent {agent} from {f} to {t}')
            if not args.batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)  # for multi agent
                # animation = Animation(my_map, [starts[0]], [goals[0]], paths) # for one agent
                # animation.save("output.mp4", 1.0)
                animation.show()
    now = datetime.now()
    dt_string = now.strftime("%d_%m_%Y_%H_%M_%S")
    wb.save(f'excels/example_{args.solver}_{dt_string}.xls')
    result_file.close()
