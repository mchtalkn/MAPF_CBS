from .my_a_star_manhattan import my_a_star as my_a_star_g
from .common_search_funcs import compute_heuristics,get_sum_of_cost
from pathmap import *
import heapq
import copy
import math
inf=math.inf
#optimized get solution step with calculating only conflicting agent's path
def get_solution(starts,goals,my_map,constraints,edge_constraints,h_values):
    paths=[]
    for agent in range(len(starts)):
        path=my_a_star_g(my_map,starts[agent],goals[agent],h_values[agent],agent,constraints,edge_constraints)
        paths.append(path)
    return paths
def cbs(my_map,starts,goals):
    heuristics=[[]]*len(starts)
    root={}
    root['constraints']=[]
    root['edge_constraints']=[]
    root['solution']=get_solution(starts,goals,my_map,[],[],heuristics)
    open_list=[]
    tie_breaker=0
    heapq.heappush(open_list,(get_sum_of_cost(root['solution']),tie_breaker,root))
    while open_list:
        #print(tie_breaker)
        _,_,node=heapq.heappop(open_list)
        #print(node)
        pm=PathMap(node['solution'])
        conflict=pm.get_one_collision()
        edge_conflict = pm.get_one_edge_collision()
        #print(f"""sol:{node['solution']}""")
        if not conflict and not edge_conflict:
            return node['solution']
        if conflict:
            (position,time),agentL=conflict
            #print("agentL:",agentL)
            for agent in agentL:
                sol=copy.deepcopy(node['solution'])
                tie_breaker+=1
                new_node={}
                new_cons=copy.deepcopy(node['constraints'])
                for tmp_agent in agentL:
                    if tmp_agent == agent:
                        continue
                    new_cons.append((tmp_agent,position,time))
                    path=my_a_star_g(my_map,starts[tmp_agent],goals[tmp_agent],heuristics[tmp_agent],tmp_agent,new_cons,node['edge_constraints'])
                    sol[tmp_agent]=path
                new_node['constraints']=new_cons
                new_node['edge_constraints']=node['edge_constraints']
                if None in sol:
                    continue
                new_node['solution']=sol
                heapq.heappush(open_list,(get_sum_of_cost(new_node['solution']),tie_breaker,new_node))
        if edge_conflict:
            (l1,l2,time),agentL=edge_conflict
            for agent in agentL:
                sol=copy.deepcopy(node['solution'])
                tie_breaker+=1
                new_node={}
                new_edge_cons=copy.deepcopy(node['edge_constraints'])
                for tmp_agent in agentL:
                    if tmp_agent == agent:
                        continue
                    new_edge_cons.append((tmp_agent,(l1,l2),time))
                    path=my_a_star_g(my_map,starts[tmp_agent],goals[tmp_agent],heuristics[tmp_agent],tmp_agent,node['constraints'],new_edge_cons)
                    sol[tmp_agent]=path
                new_node['constraints']=node['constraints']
                new_node['edge_constraints']=node['edge_constraints'] + new_edge_cons
                if None in sol:
                    continue
                new_node['solution']=sol
                heapq.heappush(open_list,(get_sum_of_cost(new_node['solution']),tie_breaker,new_node))
    return [None]*len(starts)
