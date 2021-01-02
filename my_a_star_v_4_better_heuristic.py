"""
aim of this implementation is improving heuristic by giving other paths as a
negative factor and giving last path as a positive factor
my_a_star_v_4_better_heuristic
"""
import math

from heapdict import heapdict

from pathmap import *

inf = math.inf
directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]


def is_valid_move(my_map, prev, next, constraints, edge_constraints, agent, timestep):
    x = next[0]
    y = next[1]
    return x >= 0 and x < len(my_map) and y >= 0 and y < len(my_map[0]) \
           and not my_map[x][y]


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


def my_is_constrained(agent, loc, timestep, constraints):
    return constraints.get((agent, loc, timestep)) != None


def my_is_edge_constrained(agent, prev, next, timestep, edge_constraint_table):
    return edge_constraint_table.get((agent, (prev, next), timestep)) != None


def my_build_edge_table(edge_constraints):
    table = {}
    for (curr_agent, (prev, next), time) in edge_constraints:
        table[curr_agent, (prev, next), time] = True
        table[curr_agent, (next, prev), time] = True
    return table


def my_build_constraint_table(constraints):
    table = {}
    for (curr_agent, position, time) in constraints:
        table[(curr_agent, position, time)] = True
    return table


def wait_for_constraint(agent, prev, next, timestep, constraints, edge_constraints):
    w = 0
    while my_is_constrained(agent, next, timestep, constraints) or my_is_edge_constrained(agent, prev, next, timestep,
                                                                                          edge_constraints):
        # print("waitin for constraints:",agent,next,timestep,constraints)
        if my_is_constrained(agent, prev, timestep, constraints):
            return -1
        timestep += 1
        w += 1
    return w


def my_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, edge_constraints=[], pm=PathMap([])):
    # print("starting with agent:",agent,"\nconst",constraints,"\nedge constraints",edge_constraints)
    counter = 1
    open_list = heapdict()
    closed_list = {}
    constraint_table = my_build_constraint_table(constraints)
    edge_constraint_table = my_build_edge_table(edge_constraints)
    node = {
        'prev': None,
        'location': start_loc,
        'g': 0,
        'h': h_values[start_loc[0]][start_loc[1]]
    }
    open_list[(start_loc, 0)] = (0, 0, 0, (-1, -1))
    closed_list[((-1, -1), -1)] = None
    while open_list:
        stop_flag = False
        (location, _), (f, _, h, prev) = open_list.popitem()
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
            # print(f'A* returned with {len(closed_list)} nodes')
            return path
        for direction in directions:
            new_loc = (node['location'][0] + direction[0], node['location'][1] + direction[1])
            if not is_valid_move(my_map, node['location'], new_loc, constraint_table, edge_constraint_table, agent,
                                 node['g'] + 1):
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
            if closed_node != None:
                continue
            elif open_node != None:
                continue
            else:
                # print(new_loc,"+++",g)
                counter += 1
                ng = successor['g']
                old_paths = pm.get_agents_at_position(location, g)
                l = pm.get_path_of_agent(agent)
                relative_h = (l[ng] == location) if ng < len(l) else False
                if relative_h:
                    relative_h = -1
                else:
                    relative_h = len(old_paths)
                open_list[(new_loc, successor['g'])] = (
                successor['g'] + successor['h'], relative_h, successor['h'], location)
    return None
