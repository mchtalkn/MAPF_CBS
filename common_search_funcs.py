import math
import heapq
import heapdict
def check_loc(location,my_map):
        """ check whether a location inside the path or not"""
        x=location[0]
        y=location[1]
        return (x >= 0) and (x < len(my_map)) and (y >= 0 ) and (y < len(my_map[0]))

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]

def get_sum_of_cost(paths):
    """ returns sum of costs"""
    rst = 0
    for path in paths:
        if path==None:
            continue
        rst += len(path) - 1
    return rst
def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = x = [[math.inf for i in range(len(my_map[0]))] for j in range(len(my_map))]
    for loc, node in closed_list.items():
        h_values[loc[0]][loc[1]] = node['cost']
    return h_values
