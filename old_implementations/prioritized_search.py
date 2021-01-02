from .my_a_star_v_2_heapdict import *
from .single_agent_planner import compute_heuristics

class path_map:
    paths=[]
    position_table = {}
    edge_table={}
    no_path_indexes=[]
    def __init__(self,paths):
        self.paths=[]
        self.position_table={}
        self.edge_table={}
        self.no_path_indexes=[]
        self.paths=paths
        for agent in range(len(paths)):
            path=paths[agent]
            if path != None:
                for time in range(len(path)):
                    if time == 0:
                        continue
                    if self.position_table.get((path[time],time)) == None:
                        self.position_table[(path[time],time)] = [agent]
                    else:
                        self.position_table[(path[time],time)].append(agent)
                    if self.edge_table.get((min(path[time],path[time-1]), \
                        max(path[time],path[time-1]),time)) == None:
                        self.edge_table[(min(path[time],path[time-1]), \
                            max(path[time],path[time-1]),time)] = [agent]
                    else:
                        self.edge_table[(min(path[time],path[time-1]), \
                            max(path[time],path[time-1]),time)].append(agent)
    def get_node_collisions(self):
        node_collisions=[[]]*len(self.paths)
        for key,value in self.position_table.items():
            if value != None and len(value) > 1:
                for agent in value:
                    node_collisions[agent].append(key)
        return node_collisions
    def get_edge_collisions(self):
        edge_collisions=[[]]*len(self.paths)
        for key,value in self.edge_table.items():
            if value != None and len(value) > 1:
                for agent in value:
                    edge_collisions[agent].append(key)
        return edge_collisions
    def get_collisions(self):
        collisions=[]
        for i in range(len(self.paths)):
            collisions.append([])
        for key,value  in self.position_table.items():
            if value != None and len(value) > 1:
                for i in range(len(value)):
                    agent = value[i]
                    l=value[:i]+value[i+1:]
                    collisions[agent].append(('n',key,l))
        for key,value in self.edge_table.items():
            if value != None and len(value)>1:
                for i in range(len(value)):
                    agent = value[i]
                    l=value[:i]+value[i+1:]
                    collisions[agent].append(('e',key,l))
        return collisions
    def remove_agent(self,agent):
        path=self.paths[agent]
        #print(self.position_table)
        #print(self.paths)
        for time in range(len(path)):
            if time == 0:
                continue
            self.position_table[(path[time],time)].remove(agent)
            if self.position_table[(path[time],time)] == []:
                self.position_table[(path[time],time)]=None
            if time != 0:
                self.edge_table[(min(path[time],path[time-1]), \
                    max(path[time],path[time-1]),time)].remove(agent)
                if self.edge_table[(min(path[time],path[time-1]), \
                    max(path[time],path[time-1]),time)] == []:
                    self.edge_table[(min(path[time],path[time-1]), \
                        max(path[time],path[time-1]),time)] = None
        self.paths[agent]=[]
    def get_one_collision(self):
        node_collisions=[[]]*len(self.paths)
        for key,value in self.position_table.items():
            if len(value) > 1:
                return (key,value)
        return None
    def get_one_edge_collision(self):
        for key,value in self.edge_table.items():
            if value != None and len(value) > 1:
                return (key,value)
        return None
    def change_path(self,path,agent):
        self.remove_agent(agent)
        if path != None:
            for time in range(len(path)):
                if time == 0:
                    continue
                if self.position_table.get((path[time],time)) == None:
                    self.position_table[(path[time],time)] = [agent]
                else:
                    self.position_table[(path[time],time)].append(agent)
                if self.edge_table.get((min(path[time],path[time-1]), \
                    max(path[time],path[time-1]),time)) == None:
                    self.edge_table[(min(path[time],path[time-1]), \
                        max(path[time],path[time-1]),time)] = [agent]
                else:
                    self.edge_table[(min(path[time],path[time-1]), \
                        max(path[time],path[time-1]),time)].append(agent)
            self.paths[agent]=path
def get_max_conflict(collisions):
    if not collisions:
        return None
    mc=0
    for i in range(len(collisions)):
        if len(collisions[i]) > len(collisions[mc]):
            mc = i
    return mc,len(collisions[mc])

def prioritized_search(map,starts,goals):
    paths=[]
    constraints=[]
    edge_constraints=[]
    heuristics=[]
    for agent in range(len(starts)):
        h=compute_heuristics(map,goals[agent])
        heuristics.append(h)
        path = my_a_star(map, starts[agent], goals[agent], h, agent, constraints,edge_constraints)
        paths.append(path)
    pm = path_map(paths)
    collisions = pm.get_collisions()
    agent,cc=get_max_conflict(collisions)
    while cc != 0:
       # print(paths,'\n\n\n',pm.position_table,'\n\n\n',collisions)
        conflicts=collisions[agent]
        for conflict in conflicts:
            t,key,a = conflict
            if t == 'n':
                l,t=key
                constraints.append((agent,l,t))
                #for ca in a:
                #    constraints.append((ca,l,t))
            if t == 'e':
                l1,l2,t=key
                edge_constraints.append((agent, (l1, l2), t))
                #for ca in a:
                #    edge_constraints.append((ca,(l1,l2),t))
        path= my_a_star(map, starts[agent], goals[agent], heuristics[agent], agent, constraints,edge_constraints)
        if path == None:
            print(f'No path found for agent{agent}')
            pm.change_path([],agent)
        else:
            pm.change_path(path,agent)
        collisions = pm.get_collisions()
        #print(collisions)
        agent,cc=get_max_conflict(collisions)
    return pm.paths
