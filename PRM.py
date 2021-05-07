from Node import Node, NodeTrajectory
from Robot import Robot
from Environment import Environment
from traj_planner_utils import *

import copy

import numpy as np

import random
import time
from math import sqrt, pi, floor, ceil

class PRM():
    NEIGHBOR_RADIUS = 5
    LARGE_NUMBER = float('inf')
    INIT_NODES = 100
    MAX_EXPANSIONS = 500

    def __init__(self, env):
        self.env = env
        radius = abs(env.get_walls()[0][0])
        self.xbounds = [-radius, radius]
        self.ybounds = [-radius, radius]

    """
    Learning phase.
    """
    def learning_phase(self, init_nodes=INIT_NODES, display=False):
        self.constuction_phase(num_init_nodes=init_nodes)
        self.expansion_phase()

        if display:
            # plot_roadmap(self.nodes, self.env.get_all_objects(), self.env.get_walls())
            plot_roadmap(self.nodes, self.env.get_all_objects(), self.env.get_walls())

        return self.nodes

    def constuction_phase(self, num_init_nodes=INIT_NODES):
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
    Query phase
    """
    def query(self, mode='A_star', display=False):
        if mode == 'A_star':
            traj, cost = self.A_star_plan(self.in_node)

        if display and len(traj) > 0:
            plot_roadmap(self.nodes, self.env.get_all_objects(), self.env.get_walls(), traj=traj)

        return traj, cost
    

    def initialize_query(self, s0, s1):
        self.s0 = s0
        self.s1 = s1

        # Connect the start position into the roadmap
        in_node = Node(s0)
        neighbors_in = self.find_neighbors(s0)
        self.connect_to_neighbors(in_node, neighbors_in)

        self.in_node = in_node

        # Connect the roadmap to the goal position
        goal_node = Node(s1)
        neighbors_out = self.find_neighbors(s1)
        self.connect_from_neighbors(goal_node, neighbors_out)
        
        self.goal_node = goal_node

        # Add both to the list of nodes
        self.add_to_nodes(self.goal_node)
        self.add_to_nodes(self.in_node)

        # Initialize the J of all the nodes
        self.update_all_J()


    def A_star_plan(self, start_node):
        # Start from the current node
        node_traj = NodeTrajectory(start_node)
        distance = start_node.distance(self.goal_node.state)
        fringe = [(distance, node_traj)]

        for _ in range(self.MAX_EXPANSIONS):
            if (len(fringe)) == 0:
                print("Nodes run out.")
                return [], self.LARGE_NUMBER
            
            # Expand the one with the lowest cost
            fringe.sort(key=lambda n: n[1].cost + n[0])
            dist, node_traj = fringe.pop(0)
            print(dist, node_traj.current_node.state)
            new_node_trajs = node_traj.expand_all_nodes()
            
            for node_traj in new_node_trajs:
                # Check if any of the trajectory reaches the goal
                distance = node_traj.current_node.distance(self.goal_node.state)
                if distance < 1e-1:
                    return self.construct_traj_from_node_traj(node_traj)
                
                fringe.append((distance, node_traj))

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

    """
    FIRM replanning
    """
    def update_all_J(self):
        self.goal_node.J = 0
        node_list = self.goal_node.get_back_neighbors()

        while len(node_list) > 0:
            n = node_list.pop(0)
            
            neighbors = n.get_neighbors()
            updated = False
            for neighbor in neighbors:
                path, cost = self.generate_path(n.state, neighbor.state, 0)
                J = cost + neighbor.J

                if J < n.J:
                    n.J = J
                    updated = True
                
            # Put the backneighbors to the list if this node gets updated
            if updated:
                back_neighbors = n.get_back_neighbors()
                node_list = node_list + back_neighbors
            

    def replan(self, robot, full=False):
        """
        Returns a local planner based on the current position of the robot
        """
        if full:
            self.update_all_J()
        t = robot.t
        s = robot.state

        # check if it is the end
        # path, cost = self.generate_path(s, self.goal_node.state, t)
        # if cost < self.LARGE_NUMBER:
        #     return path, cost

        distance = self.goal_node.distance(s)
        if distance < 1e-1:
            current = [t] + s
            return [current] * 2, 0

        best_J = self.LARGE_NUMBER
        neighbors = self.find_neighbors(s)
        print("Robot State: ", s)

        for neighbor in neighbors:
            path, cost = self.generate_path(s, neighbor.state, t)
            if cost >= self.LARGE_NUMBER or neighbor.J >= self.LARGE_NUMBER:
                continue
            new_J = neighbor.J + cost
            print("Neighbor State: ", neighbor.state)
            print("J ", new_J, ", cost ", cost)
        
            if new_J < best_J:
                best_J = new_J
                best_path = path
                best_cost = cost

        if best_J >= self.LARGE_NUMBER:
            return [], self.LARGE_NUMBER
        
        return best_path, best_cost

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
        # if collision_found(traj, self.env.get_all_objects(), self.env.get_walls()):
        if collision_found(traj, self.env.get_all_objects(), self.env.get_walls()):
            traj_distance = self.LARGE_NUMBER

        return traj, traj_distance

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

    def find_neighbors(self, s, radius=NEIGHBOR_RADIUS):
        neighbors = []
        for node in self.nodes:
            dist = node.distance(s)
            # ignore itself
            if dist < 1e-1:
                continue

            if dist <= radius:
                neighbors.append(node)

        return neighbors

    def connect_to_neighbors(self, node, neighbors):
        # If it has connections to a neighbor node, add the edge
        for neighbor in neighbors:
            edge, cost = self.generate_path(node.state, neighbor.state, 0)
            if cost < self.LARGE_NUMBER:
                node.add_edge(neighbor, edge, cost)
                neighbor.add_back_edge(node, edge, cost)

    def connect_from_neighbors(self, node, neighbors):
        # If it has connections from a neighbor node, add the edge to that neighbor
        for neighbor in neighbors:
            edge, cost = self.generate_path(neighbor.state, node.state, 0)
            if cost < self.LARGE_NUMBER:
                neighbor.add_edge(node, edge, cost)
                node.add_back_edge(neighbor, edge, cost)

"""
Test code
"""

def benchmark_once(env, tp0, tp1, full=False, video=False, display=False, step_display=False, mode="step"):
    robot = Robot(tp0[1:4], vel=0.5)

    # Planner
    planner = PRM(env)
    nodes = planner.learning_phase(init_nodes=200, display=False)
    planner.initialize_query(tp0[1:None], tp1[1:None])
    traj, _ = planner.query(display=display)

    live_plot = LivePlotter(env, robot, nodes, a_traj = traj, output=video, display=step_display)

    times = []
    distance = 0
    while not robot.at_location(tp1[1:4]):
        start_time = time.perf_counter()
        env.get_visible_objects(robot.state, robot.t)
        edge, edge_cost = planner.replan(robot, full=full)
        end_time = time.perf_counter()
        if edge_cost >= planner.LARGE_NUMBER:
            break
        times.append(end_time - start_time)

        moved_length = robot.move(edge, mode=mode, distance=edge_cost)
        distance += moved_length
        live_plot.plot(edge)
    
    print("Average Decision Time: ", np.mean(times))
    print("Variance: ", np.var(times))
    live_plot.generate_gif()

    return times, distance

#     Average Decision Time:  0.006549423829783627
#       Variance:  7.846492151361532e-06

    # traj, cost = planner.query(display=True)

    # if len(traj) > 0:
    #   plot_roadmap(self.nodes, self.objs, self.walls)

def benchmark(num_trials):
    t_results = []
    d_results = []
    t_results_full = []
    d_results_full = []
    for _ in range(num_trials):
        # Environment
        environment_size = 10
        num_objects = 10
        object_radius = 1
        robot_perception_radius = 10
        tp0 = [0, -8, -8, 0]
        # tp1 = [300, random.uniform(-environment_size+1, environment_size-1), random.uniform(-environment_size+1, environment_size-1), 0]
        tp1 = [300, 8, 8, 0]
        env = Environment(environment_size, num_objects, object_radius, robot_perception_radius, tp0, tp1)
        
        times, distance = benchmark_once(copy.copy(env), tp0, tp1, full=False, mode='skip')
        t_results.extend(times)
        d_results.append(distance)

        times_full, distance_full = benchmark_once(copy.copy(env), tp0, tp1, full=True, mode='skip')
        t_results_full.extend(times_full)
        d_results_full.append(distance_full)

    # Report the results
    print("Speed Performance")
    print("Average Decision Time: ", np.mean(t_results))
    print("Variance: ", np.var(t_results))
    print("Average Decision Time (full): ", np.mean(t_results_full))
    print("Variance (full): ", np.var(t_results_full))

    print("Optimality")
    print("Average Distance: ", np.mean(d_results))
    print("Variance: ", np.var(d_results))
    print("Average Distance (full): ", np.mean(d_results_full))
    print("Variance (full): ", np.var(d_results_full))


def main():
    # 1. Test the performance (speed + optimality)
    benchmark(100)

    # Environment
    environment_size = 10
    num_objects = 10
    object_radius = 1
    robot_perception_radius = 10
    tp0 = [0, -8, -8, 0]
    # tp1 = [300, random.uniform(-environment_size+1, environment_size-1), random.uniform(-environment_size+1, environment_size-1), 0]
    tp1 = [300, 8, 8, 0]
    env = Environment(environment_size, num_objects, object_radius, robot_perception_radius, tp0, tp1)

    # 2. Debug with visualization
    # benchmark_once(env, tp0, tp1, full=False, display=True, mode="skip", step_display=True)

    # 3. Generate GIF
    # benchmark_once(env, tp0, tp1, full=False, display=True, video=True, mode="step", step_display=False)


if __name__ == '__main__':
    main()