import math

from heapdict import heapdict

inf = math.inf
directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]


def is_valid_move(my_map, next_coordinate):
    x = next_coordinate[0]
    y = next_coordinate[1]
    return 0 <= x < len(my_map) and 0 <= y < len(my_map[0]) and not my_map[x][y]


def my_compute_heuristics(my_map, goal):
    a = len(my_map)
    b = len(my_map[0])
    heuristic = []
    for x in range(a):
        heuristic.append([])
        for y in range(b):
            if my_map[x][y]:
                heuristic[x].append(inf)
            else:
                heuristic[x].append(abs(x - goal[0]) + abs(y - goal[1]))
    return heuristic


def get_path(node):
    res = []
    g = node['g'] + 1
    while node:
        while g - node['g']:
            res.append(node['location'])
            g -= 1
        node = node['prev']
    res.reverse()
    # print(res)
    return res


def my_is_constrained(agent, loc, time_step, constraints):
    return constraints.get((agent, loc, time_step)) is not None


def my_is_edge_constrained(agent, prev, next_coordinate, time_step, edge_constraint_table):
    return edge_constraint_table.get((agent, (prev, next_coordinate), time_step)) is not None


def find_index_of_open(open_list, location):
    # new implementation of heap with helper dictionary and lazy deletion
    # should be used in order to make this step in o(1)
    i = 0
    for _, node in open_list:
        if node['location'] == location:
            return i
        i += 1
    return None


def my_build_edge_table(edge_constraints):
    table = {}
    for (curr_agent, (prev, next_coordinate), time) in edge_constraints:
        table[curr_agent, (prev, next_coordinate), time] = True
        table[curr_agent, (next_coordinate, prev), time] = True
    return table


def my_build_constraint_table(constraints):
    table = {}
    for (curr_agent, position, time) in constraints:
        table[(curr_agent, position, time)] = True
    return table


def wait_for_constraint(agent, prev, next_coordinate, time_step, constraints, edge_constraints):
    w = 0
    while my_is_constrained(agent, next_coordinate, time_step, constraints) or \
            my_is_edge_constrained(agent, prev, next_coordinate, time_step, edge_constraints):
        # print("waitin for constraints:",agent,next,time_step,constraints)
        if my_is_constrained(agent, prev, time_step, constraints):
            return -1
        time_step += 1
        w += 1
    return w


def my_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, edge_constraints=None):
    # print("starting with agent:",agent,"\nconst",constraints,"\nedge constraints",edge_constraints)
    if edge_constraints is None:
        edge_constraints = []
    counter = 1
    open_list = heapdict()
    closed_list = {}
    constraint_table = my_build_constraint_table(constraints)
    edge_constraint_table = my_build_edge_table(edge_constraints)
    open_list[(start_loc, 0)] = (0, 0, (-1, -1))
    closed_list[((-1, -1), -1)] = None
    while open_list:
        (location, _), (f, h, prev) = open_list.popitem()
        g = f - h
        # print(f"agent:{agent} \n open:{location}\n prev:{prev}\n f:{f} h:{h} g:{g}")
        node = {
            'location': location,
            'prev': closed_list[(prev, g - 1)],
            'g': g,
            'h': h_values[location[0]][location[1]]
        }
        closed_list[(location, g)] = node
        if location == goal_loc:
            path = get_path(node)
            return path
        for direction in directions:
            new_loc = (node['location'][0] + direction[0], node['location'][1] + direction[1])
            if not is_valid_move(my_map, new_loc):
                continue
            w = wait_for_constraint(agent, node['location'], new_loc, node['g'] + 1, constraint_table,
                                    edge_constraint_table)
            if w == -1:
                continue
            for i in range(w):
                tmp_node = {
                    'location': location,
                    'prev': closed_list[(location, g + i)],
                    'g': node['g'] + i + 1,
                    'h': h_values[location[0]][location[1]]
                }
                closed_list[(location, tmp_node['g'])] = tmp_node
            successor = {
                'prev': node,
                'location': new_loc,
                'g': node['g'] + w + 1,
                'h': h_values[new_loc[0]][new_loc[1]]
            }
            closed_node = closed_list.get((new_loc, successor['g']))
            open_node = open_list.get((new_loc, successor['g']))
            if successor['h'] == math.inf:
                continue
            if closed_node is not None:
                continue
            elif open_node is not None:
                continue
            else:
                # print(new_loc,"+++",g)
                counter += 1
                open_list[(new_loc, successor['g'])] = (successor['g'] + successor['h'], successor['h'], location)
    return None
