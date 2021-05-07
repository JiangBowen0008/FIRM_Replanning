import math
import random

class Node():
    def __init__(self, state):
        self.state = state
        self.edges = []
        self.neighbors = []
        self.connections = {}

        self.back_neighbors = []
        self.back_edges = []
        self.back_connections = {}
        self.J = float('inf')
    
    def add_edge(self, node_rhs, edge, cost):
        self.connections[tuple(node_rhs.state)] = (edge, cost)
        self.edges.append(edge)
        self.neighbors.append(node_rhs)

    def add_back_edge(self, node_rhs, back_edge, cost):
        self.back_connections[tuple(node_rhs.state)] = (back_edge, cost)
        self.back_edges.append(back_edge)
        self.back_neighbors.append(node_rhs)

    """
    Forward Utilities
    """
    
    def get_edges(self):
        return self.edges

    def get_neighbors(self):
        return self.neighbors

    def get_neighbor_path(self, neighbor):
        key = tuple(neighbor.state)

        if key not in self.connections.keys():
            return [], float('inf')
        
        return self.connections[key]

    """
    Backwards Tracking Utilities
    """

    def get_back_edges(self):
        return self.back_edges

    def get_back_neighbors(self):
        return self.back_neighbors

    def get_back_neighbor_path(self, neighbor):
        key = tuple(neighbor.state)

        if key not in self.back_connections.keys():
            return [], float('inf')
        
        return self.back_connections[key]

    def distance(self, state):
        return math.sqrt( (self.state[0] - state[0])**2 + (self.state[1] - state[1])**2 )

class NodeTrajectory():
    def __init__(self, node_start, traj=[], cost = 0):
        self.current_node = node_start
        if len(traj) == 0:
            traj = [node_start]
        self.node_traj = traj
        self.cost = cost
    
    def expand_all_nodes(self):
        new_node_trajs = []

        neighbors = self.current_node.get_neighbors()
        for neighbor in neighbors:
            _, cost = self.current_node.get_neighbor_path(neighbor)
            new_node_traj = self.node_traj + [neighbor]
            new_cost = self.cost + cost

            new_node_traj = NodeTrajectory(neighbor, new_node_traj, new_cost)
            new_node_trajs.append(new_node_traj)
        
        return new_node_trajs 

    def expand_random_node(self):
        neighbors = self.current_node.get_neighbors()
        neighbor = random.choice(neighbors)

        _, cost = self.current_node.get_neighbor_path(neighbor)
        new_node_traj = self.node_traj + [neighbor]
        new_cost = self.cost + cost

        new_node_traj = NodeTrajectory(neighbor, new_node_traj, new_cost)
        
        return new_node_traj


