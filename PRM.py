import numpy as np

class PRM():
    NEIGHBOR_RADIUS = 2

    def __init__(self, walls, objs):
        self.walls = walls
        self.objs = objs
        pass

    """
    Learning phase.
    """
    def learning_phase(self):
        self.constuction_phase()
        pass

    def constuction_phase(self):
        pass

    def expansion_phase(self):
        pass
    
    """
    Query phase - FIRM replanning
    """
    def initialize_query(self):
        pass

    def global_plan(self):
        pass

    def replan(self):
        pass
        neighbors = find_neighbors(b0)

        best_J = LARGE_NUMBER

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
    def generate_path(self, s0, s1):
        pass


    def find_neighbors(self, s, radius=NEIGHBOR_RADIUS):
        pass