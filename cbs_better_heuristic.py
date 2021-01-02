"""
aim of this implementation is improving heuristic by giving other paths as a
negative factor and giving last path as a positive factor
see also my_a_star_v_4_better_heuristic
"""
from my_a_star_v_4_better_heuristic import my_a_star as my_a_star_g
from common_search_funcs import compute_heuristics, get_sum_of_cost
from pathmap import *
import heapq
import copy
import math

inf = math.inf


# optimized get solution step with calculating only conflicting agent's path
def get_solution(starts, goals, my_map, constraints, edge_constraints, h_values):
    """ computes solutions for all agents """
    paths = []
    pm = PathMap([[]] * len(starts))
    for agent in range(len(starts)):
        path = my_a_star_g(my_map, starts[agent], goals[agent], h_values[agent], agent, constraints, edge_constraints,
                           pm)
        paths.append(path)
    return paths


def cbs(my_map, starts, goals):
    heuristics = []
    for agent in range(len(starts)):
        h = compute_heuristics(my_map, goals[agent])
        heuristics.append(h)
    root = {'constraints': [], 'edge_constraints': [],
            'solution': get_solution(starts, goals, my_map, [], [], heuristics)}
    open_list = []
    tie_breaker = 0
    node_count = 0
    heapq.heappush(open_list, (get_sum_of_cost(root['solution']), tie_breaker, root))
    while open_list:
        node_count += 1
        # print(tie_breaker)
        _, _, node = heapq.heappop(open_list)
        # print(node)
        pm = PathMap(node['solution'])
        conflict = pm.get_one_collision()
        edge_conflict = pm.get_one_edge_collision()
        # print(f"""sol:{node['solution']}""")
        if not conflict and not edge_conflict:
            print(f"cbs returned with a solution with {tie_breaker} nodes")
            return node['solution'], tie_breaker
        if conflict:
            """if there is conflict between paths in a location, try to solve it """
            (position, time), agent_l = conflict
            for agent in agent_l:
                solution_success_flag = True
                sol = copy.deepcopy(node['solution'])
                tie_breaker += 1
                new_node = {}
                new_cons = copy.deepcopy(node['constraints'])
                for tmp_agent in agent_l:
                    if tmp_agent == agent:
                        continue
                    new_cons.append((tmp_agent, position, time))
                    path = my_a_star_g(my_map, starts[tmp_agent], goals[tmp_agent], heuristics[tmp_agent], tmp_agent,
                                       new_cons, node['edge_constraints'], pm)
                    if path is None:
                        solution_success_flag = False
                        break
                    sol[tmp_agent] = path
                if not solution_success_flag:
                    continue
                new_node['constraints'] = new_cons
                new_node['edge_constraints'] = node['edge_constraints']
                new_node['solution'] = sol
                heapq.heappush(open_list, (get_sum_of_cost(new_node['solution']), tie_breaker, new_node))
        if edge_conflict:
            """if there is conflict between paths in an edge, try to solve it """
            (l1, l2, time), agent_l = edge_conflict
            for agent in agent_l:
                sol = copy.deepcopy(node['solution'])
                tie_breaker += 1
                new_node = {}
                new_edge_cons = copy.deepcopy(node['edge_constraints'])
                for tmp_agent in agent_l:
                    if tmp_agent == agent:
                        continue
                    new_edge_cons.append((tmp_agent, (l1, l2), time))
                    path = my_a_star_g(my_map, starts[tmp_agent], goals[tmp_agent], heuristics[tmp_agent], tmp_agent,
                                       node['constraints'], new_edge_cons, pm)
                    sol[tmp_agent] = path
                new_node['constraints'] = node['constraints']
                new_node['edge_constraints'] = node['edge_constraints'] + new_edge_cons
                if None in sol:
                    continue
                new_node['solution'] = sol
                heapq.heappush(open_list, (get_sum_of_cost(new_node['solution']), tie_breaker, new_node))
    return [None] * len(starts), tie_breaker
