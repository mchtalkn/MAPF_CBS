import heapq
import math
import copy

inf=math.inf
directions=[(0,1),(1,0),(0,-1),(-1,0),(0,0)]
def is_valid_move(my_map,prev ,next,constraints,edge_constraints,agent,timestep):
    x=next[0]
    y=next[1]
    return (not my_is_constrained(agent,next,timestep,constraints) \
        and not my_is_edge_constrained(agent,prev,next,timestep,edge_constraints)
        and x >= 0 and x < len(my_map) and y >= 0 and y < len(my_map[0]) \
        and not my_map[x][y])
def my_compute_heuristics(my_map, goal):
    a = len(my_map)
    b = len(my_map[0])
    heuristic=[]
    for x in range(a):
        heuristic.append([])
        for y in range(b):
            if my_map[x][y]:
                heuristic[x].append(inf)
            else:
             #    print('else:',abs(x-goal[0])+abs(y-goal[1]))
                heuristic[x].append(abs(x-goal[0])+abs(y-goal[1]))
    return heuristic
def get_path(node):
    res=[]
    while node:
        res.append(node['location'])
        node=node['prev']
    res.reverse()
    return res
def my_is_constrained(agent,loc,timestep,constraints):
    return constraints.get((agent,loc,timestep)) != None
def my_is_edge_constrained(agent,prev,next,timestep,edge_constraint_table):
    return edge_constraint_table.get((agent,(prev,next),timestep)) != None
def find_index_of_open(open_list,location):
    # new implementation of heap with helper dictionary and lazy deletion
    # should be used in order to make this step in o(1)
    i = 0
    for _,node in open_list:
        if node['location']== location:
            return i
        i+=1
    return None
def my_build_edge_table(edge_constraints):
    table={}
    for(curr_agent,(prev,next),time) in edge_constraints:
        table[curr_agent,(prev,next),time]=True
        table[curr_agent,(next,prev),time]=True
    return table
def my_build_constraint_table(constraints):
    table={}
    for (curr_agent,position,time) in constraints:
        table[(curr_agent,position,time)]=True
    return table
def my_a_star(my_map, start_loc, goal_loc, h_values, agent, constraints,edge_constraints=[]):
    tie_breaker= 1
    open_list=[]
    closed_list={}
    constraint_table = my_build_constraint_table(constraints)
    edge_constraint_table = my_build_edge_table(edge_constraints)
    node={
    'prev':None,
    'location':start_loc,
    'g':0,
    'h':h_values[start_loc[0]][start_loc[1]]
    }
    heapq.heappush(open_list,((node['g']+node['h'],node['h'],start_loc,0),node))
    while open_list:
        _,node=heapq.heappop(open_list)
        if node == None:
            continue
        if node['location']==goal_loc:
            path= get_path(node)
            return get_path(node)
        #print('node:',node['location'])
        closed_list[node['location']]=node
        for direction in directions:
            new_loc=(node['location'][0]+direction[0],node['location'][1]+direction[1])
            if not is_valid_move(my_map,node['location'],new_loc,constraint_table,edge_constraint_table,agent,node['g']+1):
                continue
            successor = {
            'prev' : node,
            'location' : new_loc,
            'g': node['g']+1,
            'h':h_values[new_loc[0]][new_loc[1]]
            }
            closed_node = closed_list.get(new_loc)
            open_index = find_index_of_open(open_list,new_loc)
            if closed_node != None:
                if closed_node['g'] > successor['g']:
                    closed_list[new_loc] = successor
                    heapq.heappush(open_list,((successor['g'] + successor['h'],node['h'],new_loc,tie_breaker),successor))
                """elif open_index != None :
                    open_node = open_list[open_index][1]
                    if open_node['g'] <= successor['g'] :
                        continue
                    else:
                        del open_list[open_index]
                        heapq.heapify(open_list)
                        heapq.heappush(open_list,((successor['g'] + successor['h'],node['h'],new_loc,tie_breaker),successor))
                        tie_breaker+=1"""
            else:
                heapq.heappush(open_list,((successor['g'] + successor['h'],successor['h'],new_loc,tie_breaker),successor))
                tie_breaker+=1


    return None
