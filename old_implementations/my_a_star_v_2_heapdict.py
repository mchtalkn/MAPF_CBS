import heapq
import math
import copy
from heapdict import heapdict
inf=math.inf
directions=[(0,1),(1,0),(0,-1),(-1,0)]
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
    g=node['g']+1
    while node:
        while g-node['g']:
            res.append(node['location'])
            g-=1
        node=node['prev']
    res.reverse()
    #print(res)
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
    #print("starting with",constraints,"edge constraints",edge_constraints)
    tie_breaker= 1
    open_list=heapdict()
    closed_list={}
    constraint_table = my_build_constraint_table(constraints)
    edge_constraint_table = my_build_edge_table(edge_constraints)
    node={
    'prev':None,
    'location':start_loc,
    'g':0,
    'h':h_values[start_loc[0]][start_loc[1]]
    }
    open_list[start_loc]=(0,0,(-1,-1))
    closed_list[(-1,-1)] = None
    while open_list:
        location,(f,g,prev)=open_list.popitem()
        node={
        'location' : location,
        'prev' : closed_list[prev],
        'g' : g,
        'h' : h_values[location[0]][location[1]]
        }
        closed_list[location]=node
        if  location == goal_loc:
            path= get_path(node)
            return path
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
            open_node = open_list.get(new_loc)
            if closed_node != None:
                if closed_node['g'] > successor['g']:
                    closed_list[new_loc] = successor
                    open_list[new_loc] = successor
            elif open_node != None:
                f,g,prev = open_node
                if g <= successor['g'] :
                    continue
                else:
                    open_list[new_loc] = (successor['g']+successor['h'],successor['g'],location)

            else:
                open_list[new_loc] = (successor['g']+successor['h'],successor['g'],location)
    return None
