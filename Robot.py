from math import sqrt

class Robot():
    """
    Robot mover
    """

    def __init__(self, s0, vel=0.5):
        self.state = s0
        self.t = 0
        self.vel = vel
        self.traj = [s0]

    def move(self, edge, mode="step", distance=None):
        if mode == "step":
            goal = edge[1][1:4]
            distance = sqrt((goal[0] - self.state[0])**2 + (goal[1] - self.state[1])**2)
        elif mode == "skip":
            goal = edge[-1][1:4]
            assert distance is not None

        # Calculates the time taken to travel along the edge
        delta_t = distance / self.vel
        self.t += delta_t

        # Moves the robot
        self.state = goal
        self.traj.append(goal)

        return distance

    def at_location(self, loc):
        distance = sqrt((loc[0] - self.state[0])**2 + (loc[1] - self.state[1])**2)
        if distance < 1e-1:
            return True
        
        return False
