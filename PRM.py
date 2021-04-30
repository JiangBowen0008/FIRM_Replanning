from Node import Node, NodeTrajectory
from traj_planner_utils import *

import numpy as np

import random
from math import sqrt, pi, floor, ceil

class PRM():
    NEIGHBOR_RADIUS = 5
    LARGE_NUMBER = float('inf')
    MAX_EXPANSIONS = 100000

    def __init__(self, walls, objs):
        self.walls = walls
        self.objs = objs
        radius = abs(walls[0][0])
        self.xbounds = [-radius, radius]
        self.ybounds = [-radius, radius]

    """
    Learning phase.
    """
    def learning_phase(self, init_nodes=50, display=False):
        self.constuction_phase(num_init_nodes=init_nodes)
        self.expansion_phase()

        if display:
            plot_roadmap(self.nodes, self.objs, self.walls)

    def constuction_phase(self, num_init_nodes=50):
        self.nodes = []
        self.edges = {}

        for i in range(num_init_nodes):
            node = self.generate_random_node()
            neighbors = self.find_neighbors(node.state)
            self.connect_to_neighbors(node, neighbors)
            self.connect_from_neighbors(node, neighbors)
            self.add_to_nodes(node)

            
    def expansion_phase(self):
        pass
    
    """
    Query phase - FIRM replanning
    """
    def query(self, mode='A_star', display=False):
        if mode == 'A_star':
            traj, cost = self.A_star_plan()

        if display and len(traj) > 0:
            plot_roadmap(self.nodes, self.objs, self.walls, traj=traj)
    

    def initialize_query(self, s0, s1):
        self.s0 = s0
        self.s1 = s1

        # Connect the start position into the roadmap
        in_node = Node(s0)
        neighbors_in = self.find_neighbors(s0)
        self.connect_to_neighbors(in_node, neighbors_in)

        # Connect the roadmap to the goal position
        goal_node = Node(s1)
        neighbors_out = self.find_neighbors(s1)
        self.connect_from_neighbors(goal_node, neighbors_out)
        
        self.goal_node = goal_node

        # Start from the current node
        node_traj = NodeTrajectory(in_node)
        distance = in_node.distance(goal_node.state)
        self.fringe = [(distance, node_traj)]


    def A_star_plan(self):
        for _ in range(self.MAX_EXPANSIONS):
            if (len(self.fringe)) == 0:
                print("Nodes run out.")
                return [], self.LARGE_NUMBER
            
            # Expand the one with the lowest cost
            self.fringe.sort(key=lambda n: n[1].cost + n[0])
            dist, node_traj = self.fringe.pop(0)
            print(dist, node_traj.current_node.state)
            new_node_trajs = node_traj.expand_all_nodes()

            for node_traj in new_node_trajs:
                # Check if any of the trajectory reaches the goal
                distance = node_traj.current_node.distance(self.goal_node.state)
                if distance < 1e-1:
                    return self.construct_traj_from_node_traj(node_traj)
                
                self.fringe.append((distance, node_traj))        
        
        return [], self.LARGE_NUMBER

    def expensive_plan(self):
        for _ in range(self.MAX_EXPANSIONS):
            print(len(self.fringe))
            if (len(self.fringe)) == 0:
                print("Nodes run out.")
                return [], self.LARGE_NUMBER
            
            # Expand the one with the lowest cost
            node_traj = random.choice(self.fringe)
            new_node_traj = node_traj.expand_random_node()

            # Check if the new trajectory reaches the goal
            distance = new_node_traj.current_node.distance(self.goal_node.state)
            if distance < 1e-1:
                return self.construct_traj_from_node_traj(new_node_traj)
            
            self.fringe.append((distance, node_traj))
        
        return [], self.LARGE_NUMBER

    
    def construct_traj_from_node_traj(self, N):
        total_cost = 0.0
        time = 0.0
        traj = []
        print(N.node_traj)
        for i in range(1, len(N.node_traj)):
            node0 = N.node_traj[i-1]
            node1 = N.node_traj[i]
            path, cost = self.generate_path(node0.state, node1.state, time)
            traj.extend(path)

            total_cost += cost
            time = traj[-1][0]
        
        print(len(traj))
        return traj, total_cost
            
    def replan(self):
        pass
        neighbors = find_neighbors(b0)

        best_J = self.LARGE_NUMBER

        for neighbor in neighbors:
            path, cost, prob = generate_path(b0, neighbor)
            # notice that prob here is a list of probabilities of ending
            # on different nodes.
            # prob[0, :] is the index of the end nodes
            # prob[1, :] is the probabilities
            new_J = cost + np.sum(prob[1] * J[prob[0]])
        
            if new_J < best_J:
                best_J = new_J
                best_path = path
        
        return best_path


    """
    Util functions
    """
    def generate_random_node(self)->Node:
        pos_x = np.random.random(1)*(self.xbounds[1] - self.xbounds[0]) + self.xbounds[0]
        pos_y = np.random.random(1)*(self.ybounds[1] - self.ybounds[0]) + self.ybounds[0]
        angle = np.random.random(1)*2*pi - pi

        node = Node(pos_x.tolist() + pos_y.tolist() + angle.tolist())
        return node

    def add_to_nodes(self, node):
        self.nodes.append(node)

    # def add_to_edges(self, node0, node1, edge):
    #     key0 = node0.state
    #     key1 = node1.state
    #     key = (key0, key1)
    #     self.edges[key] = edge

    def generate_path(self, s0, s1, t0):
        traj, traj_distance = construct_dubins_traj(s0, s1, t0)

        # Check collision
        if collision_found(traj, self.objs, self.walls):
            traj_distance = self.LARGE_NUMBER

        return traj, traj_distance


    def find_neighbors(self, s, radius=NEIGHBOR_RADIUS):
        neighbors = []
        for node in self.nodes:
            dist = node.distance(s)
            if dist <= radius:
                neighbors.append(node)

        return neighbors

    def connect_to_neighbors(self, node, neighbors):
        # If it has connections to a neighbor node, add the edge
        for neighbor in neighbors:
            edge, cost = self.generate_path(node.state, neighbor.state, 0)
            if cost < self.LARGE_NUMBER:
                node.add_edge(neighbor, edge, cost)

    def connect_from_neighbors(self, node, neighbors):
        # If it has connections from a neighbor node, add the edge to that neighbor
        for neighbor in neighbors:
            edge, cost = self.generate_path(neighbor.state, node.state, 0)
            if cost < self.LARGE_NUMBER:
                neighbor.add_edge(node, edge, cost)

"""
Test code
"""
def main():
    # Environment
    maxR = 10
    tp0 = [0, -8, -8, 0]
    tp1 = [300, random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0]
    
    walls = [[-maxR, maxR, maxR, maxR, 2*maxR], [maxR, maxR, maxR, -maxR, 2*maxR], [maxR, -maxR, -maxR, -maxR, 2*maxR], [-maxR, -maxR, -maxR, maxR, 2*maxR] ]
    num_objects = 25
    objects = []
    for j in range(0, num_objects): 
      obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      while (abs(obj[0]-tp0[1]) < 1 and abs(obj[1]-tp0[2]) < 1) or (abs(obj[0]-tp1[1]) < 1 and abs(obj[1]-tp1[2]) < 1):
        obj = [random.uniform(-maxR+1, maxR-1), random.uniform(-maxR+1, maxR-1), 0.5]
      objects.append(obj)

    # Planner
    planner = PRM(walls, objects)
    planner.learning_phase(init_nodes=100, display=True)
    # traj = planner.construct_traj(tp0, tp1, objects, walls)
    # if len(traj) > 0:
    #   plot_traj(traj, traj, objects, walls)

    planner.initialize_query(tp0[1:None], tp1[1:None])
    traj, cost = planner.query(display=True)

    # if len(traj) > 0:
    #   plot_roadmap(self.nodes, self.objs, self.walls)

if __name__ == '__main__':
    main()