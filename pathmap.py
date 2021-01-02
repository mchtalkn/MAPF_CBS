# noinspection PySimplifyBooleanCheck
class PathMap:
    """ this class is used for check conflicts between paths"""
    paths = []
    position_table = {}
    edge_table = {}
    no_path_indexes = []

    def __init__(self, paths):
        self.paths = []
        self.position_table = {}
        self.edge_table = {}
        self.no_path_indexes = []
        self.paths = paths
        for agent in range(len(paths)):
            path = paths[agent]
            if path is not None:
                for time in range(len(path)):
                    if time == 0:
                        continue
                    if self.position_table.get((path[time], time)) is None:
                        self.position_table[(path[time], time)] = [agent]
                    else:
                        self.position_table[(path[time], time)].append(agent)
                    if self.edge_table.get((min(path[time], path[time - 1]),
                                            max(path[time], path[time - 1]), time)) is None:
                        self.edge_table[(min(path[time], path[time - 1]),
                                         max(path[time], path[time - 1]), time)] = [agent]
                    else:
                        self.edge_table[(min(path[time], path[time - 1]),
                                         max(path[time], path[time - 1]), time)].append(agent)

    def get_node_collisions(self):
        node_collisions = [[]] * len(self.paths)
        for key, value in self.position_table.items():
            if value is not None and len(value) > 1:
                for agent in value:
                    node_collisions[agent].append(key)
        return node_collisions

    def get_edge_collisions(self):
        edge_collisions = [[]] * len(self.paths)
        for key, value in self.edge_table.items():
            if value is not None and len(value) > 1:
                for agent in value:
                    edge_collisions[agent].append(key)
        return edge_collisions

    def get_collisions(self):
        collisions = []
        for i in range(len(self.paths)):
            collisions.append([])
        for key, value in self.position_table.items():
            if value is not None and len(value) > 1:
                for i in range(len(value)):
                    agent = value[i]
                    l = value[:i] + value[i + 1:]
                    collisions[agent].append(('n', key, l))
        for key, value in self.edge_table.items():
            if value is not None and len(value) > 1:
                for i in range(len(value)):
                    agent = value[i]
                    l = value[:i] + value[i + 1:]
                    collisions[agent].append(('e', key, l))
        return collisions

    def remove_agent(self, agent):
        path = self.paths[agent]
        # print(self.position_table)
        # print(self.paths)
        for time in range(len(path)):
            if time == 0:
                continue
            self.position_table[(path[time], time)].remove(agent)
            if self.position_table[(path[time], time)] == []:
                self.position_table[(path[time], time)] = None
            if time != 0:
                self.edge_table[(min(path[time], path[time - 1]), max(path[time], path[time - 1]), time)].remove(agent)
                if self.edge_table[(min(path[time], path[time - 1]), max(path[time], path[time - 1]), time)] == []:
                    self.edge_table[(min(path[time], path[time - 1]), max(path[time], path[time - 1]), time)] = None
        self.paths[agent] = []

    def get_one_collision(self):
        node_collisions = [[]] * len(self.paths)
        for key, value in self.position_table.items():
            if len(value) > 1:
                return key, value
        return None

    def get_one_edge_collision(self):
        for key, value in self.edge_table.items():
            if value is not None and len(value) > 1:
                return key, value
        return None

    def change_path(self, path, agent):
        self.remove_agent(agent)
        if path is not None:
            for time in range(len(path)):
                if time == 0:
                    continue
                if self.position_table.get((path[time], time)) is None:
                    self.position_table[(path[time], time)] = [agent]
                else:
                    self.position_table[(path[time], time)].append(agent)
                if self.edge_table.get((min(path[time], path[time - 1]),
                                        max(path[time], path[time - 1]), time)) is None:
                    self.edge_table[(min(path[time], path[time - 1]), max(path[time], path[time - 1]), time)] = [agent]
                else:
                    self.edge_table[(min(path[time], path[time - 1]),
                                     max(path[time], path[time - 1]), time)].append(agent)
            self.paths[agent] = path

    def get_agents_at_position(self, location, time):
        res = self.position_table.get((location, time))
        return res if res else []

    def get_path_of_agent(self, agent):
        return self.paths[agent]


def get_max_conflict(collisions):
    if not collisions:
        return None
    mc = 0
    for i in range(len(collisions)):
        if len(collisions[i]) > len(collisions[mc]):
            mc = i
    return mc, len(collisions[mc])
