# E206 Motion Planning

# Simple planner
# C Clark

import math
from datetime import datetime
import os
import dubins
import imageio
import numpy as np
import matplotlib.pyplot as plt

DISTANCE_STEP_SIZE = 0.1 #m
COLLISION_INDEX_STEP_SIZE = 5
ROBOT_RADIUS = 0.4 #m
TURNING_RADIUS = 0.5
EDGE_VEL = 1.0


class LivePlotter():
  def __init__(self, env, robot, nodes, dir="trials", a_traj = None, output=False, display=False):
    self.env = env
    self.robot = robot
    self.nodes = nodes
    self.a_traj = a_traj
    self.output = output
    self.display = display

    if output:
      now = datetime.now()
      current_time = now.strftime("%H_%M_%S")
      self.dir = os.path.join(dir, current_time)
      os.makedirs(self.dir, exist_ok=False)

  def generate_gif(self):
    if self.output:
      img_list = os.listdir(self.dir)
      img_list = [os.path.splitext(path)[0] for path in img_list]
      img_list.sort(key=lambda s: float(s))

      images = []
      for img in img_list:
        filename = os.path.join(self.dir, img+".png")
        images.append(imageio.imread(filename))

      gif_name = os.path.join(self.dir, 'movie.gif')
      imageio.mimsave(gif_name, images)

  def plot(self, edge=None):
    if (not self.display) and (not self.output):
      return
    
    EDGE_COLOR = '0.8'

    objects = self.env.get_all_objects()
    walls = self.env.get_walls()

    fig, axis_array = plt.subplots(1)
    axis_array = [axis_array, None]
    time_stamp_desired = []
    x_desired = []
    y_desired = []
    theta_desired = []

    # Plot the robot
    axis_array[0].plot(self.robot.state[0], self.robot.state[1], 'C0x')

    # Plots the nodes
    for node in self.nodes:
      axis_array[0].plot(node.state[0], node.state[1], color='lightgray', marker='x')

    # time_stamp_actual = []
    # x_actual = []
    # y_actual = []
    # theta_actual = []
    # for tp in traj_actual:
    #   time_stamp_actual.append(tp[0])
    #   x_actual.append(tp[1])
    #   y_actual.append(tp[2])
    #   theta_actual.append(angle_diff(tp[3]))
    # axis_array[0].plot(x_actual, y_actual, 'k')

    ang_res = 0.2
    for o in objects:
      x_obj = []
      y_obj = []
      ang = 0
      while ang < 6.28:
        x_obj.append(o[0]+o[2]*math.cos(ang))
        y_obj.append(o[1]+o[2]*math.sin(ang))
        ang += ang_res
      x_obj.append(x_obj[0])
      y_obj.append(y_obj[0])
      axis_array[0].plot(x_obj, y_obj, 'k')
    for w in walls:
      axis_array[0].plot([w[0], w[2]], [w[1], w[3]], 'k')

    if self.a_traj is not None:
      if len(self.a_traj) > 0:
        x_, y_ = [], []
        for tp in self.a_traj:
          time_stamp_desired.append(tp[0])
          x_.append(tp[1])
          y_.append(tp[2])
          theta_desired.append(angle_diff(tp[3]))
        axis_array[0].plot(x_, y_, 'C0')
        axis_array[0].plot(x_[0], y_[0], 'ko')
        axis_array[0].plot(x_[-1], y_[-1], 'kx')

    if edge is not None:
      x_, y_ = [], []
      for tp in edge:
        time_stamp_desired.append(tp[0])
        x_.append(tp[1])
        y_.append(tp[2])
        theta_desired.append(angle_diff(tp[3]))
      axis_array[0].plot(x_, y_, 'C1')
      axis_array[0].plot(x_[0], y_[0], 'ko')
      axis_array[0].plot(x_[-1], y_[-1], 'kx')

    axis_array[0].set_xlabel('X (m)')
    axis_array[0].set_ylabel('Y (m)')
    axis_array[0].axis('equal')

    if self.output:
      filename = os.path.join(self.dir, str(round(self.robot.t, 1))+".png")
      plt.savefig(filename)
      plt.close()
    elif self.display:
      plt.show()

    
  

def construct_dubins_traj(start, end, t_start):
  """ Construct a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Returns:
        traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        traj_distance (float): The length ofthe trajectory (m).
  """
  q0 = tuple(start)
  q1 = tuple(end)
  turning_radius = TURNING_RADIUS
  step_size = DISTANCE_STEP_SIZE

  path = dubins.shortest_path(q0, q1, turning_radius)
  configurations, distances = path.sample_many(step_size)

  distances = np.array(distances)
  distances = np.expand_dims(distances, axis=1)
  t =  distances/EDGE_VEL + t_start
  configurations = np.array(configurations)
  traj = (np.concatenate((t, configurations), axis=1)).tolist()
  traj_distance = distances[-1, 0]

  return traj, traj_distance


def plot_traj(traj_desired, traj_actual, objects, walls):
  """ Plot a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Arguments:
        desired_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        actual_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of stationay object states with X, Y, radius (m, m, m).
        walls (list of lists: A list of walls with corners X1, Y1 and X2, Y2 points, length (m, m, m, m, m).
  """
  fig, axis_array = plt.subplots(2,1)
  time_stamp_desired = []
  x_desired = []
  y_desired = []
  theta_desired = []
  for tp in traj_desired:
    time_stamp_desired.append(tp[0])
    x_desired.append(tp[1])
    y_desired.append(tp[2])
    theta_desired.append(angle_diff(tp[3]))
  axis_array[0].plot(x_desired, y_desired, 'b')
  axis_array[0].plot(x_desired[0], y_desired[0], 'ko')
  axis_array[0].plot(x_desired[-1], y_desired[-1], 'kx')
  time_stamp_actual = []
  x_actual = []
  y_actual = []
  theta_actual = []
  for tp in traj_actual:
    time_stamp_actual.append(tp[0])
    x_actual.append(tp[1])
    y_actual.append(tp[2])
    theta_actual.append(angle_diff(tp[3]))
  axis_array[0].plot(x_actual, y_actual, 'k')

  ang_res = 0.2
  for o in objects:
    x_obj = []
    y_obj = []
    ang = 0
    while ang < 6.28:
      x_obj.append(o[0]+o[2]*math.cos(ang))
      y_obj.append(o[1]+o[2]*math.sin(ang))
      ang += ang_res
    x_obj.append(x_obj[0])
    y_obj.append(y_obj[0])
    axis_array[0].plot(x_obj, y_obj, 'b')
  for w in walls:
    axis_array[0].plot([w[0], w[2]], [w[1], w[3]], 'k')
  axis_array[0].set_xlabel('X (m)')
  axis_array[0].set_ylabel('Y (m)')
  axis_array[0].axis('equal')
  
  axis_array[1].plot(time_stamp_desired, x_desired,'b')
  axis_array[1].plot(time_stamp_desired, y_desired,'b--')
  axis_array[1].plot(time_stamp_desired, theta_desired,'b-.')
  axis_array[1].plot(time_stamp_actual, x_actual,'k')
  axis_array[1].plot(time_stamp_actual, y_actual,'k--')
  axis_array[1].plot(time_stamp_actual, theta_actual,'k-.')
  axis_array[1].set_xlabel('Time (s)')
  axis_array[1].legend(['X Desired (m)', 'Y Desired (m)', 'Theta Desired (rad)', 'X (m)', 'Y (m)', 'Theta (rad)'])

  plt.show()


def plot_roadmap(nodes, objects, walls, traj=None):
  """ Plot a trajectory in the X-Y space and in the time-X,Y,Theta space.
      Arguments:
        desired_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        actual_traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of stationay object states with X, Y, radius (m, m, m).
        walls (list of lists: A list of walls with corners X1, Y1 and X2, Y2 points, length (m, m, m, m, m).
  """
  EDGE_COLOR = '0.8'


  fig, axis_array = plt.subplots(1)
  axis_array = [axis_array, None]
  time_stamp_desired = []
  x_desired = []
  y_desired = []
  theta_desired = []
  for node in nodes:
    for edge in node.get_edges():
      line_x = []
      line_y = [] 
      for s in edge:
        line_x.append(s[1])
        line_y.append(s[2])
      axis_array[0].plot(line_x, line_y, EDGE_COLOR, '-')

    axis_array[0].plot(node.state[0], node.state[1], 'C0x')

  # time_stamp_actual = []
  # x_actual = []
  # y_actual = []
  # theta_actual = []
  # for tp in traj_actual:
  #   time_stamp_actual.append(tp[0])
  #   x_actual.append(tp[1])
  #   y_actual.append(tp[2])
  #   theta_actual.append(angle_diff(tp[3]))
  # axis_array[0].plot(x_actual, y_actual, 'k')

  ang_res = 0.2
  for o in objects:
    x_obj = []
    y_obj = []
    ang = 0
    while ang < 6.28:
      x_obj.append(o[0]+o[2]*math.cos(ang))
      y_obj.append(o[1]+o[2]*math.sin(ang))
      ang += ang_res
    x_obj.append(x_obj[0])
    y_obj.append(y_obj[0])
    axis_array[0].plot(x_obj, y_obj, 'k')
  for w in walls:
    axis_array[0].plot([w[0], w[2]], [w[1], w[3]], 'k')

  if traj is not None:
    x_, y_ = [], []
    for tp in traj:
      time_stamp_desired.append(tp[0])
      x_.append(tp[1])
      y_.append(tp[2])
      theta_desired.append(angle_diff(tp[3]))
    axis_array[0].plot(x_, y_, 'C1')
    axis_array[0].plot(x_[0], y_[0], 'ko')
    axis_array[0].plot(x_[-1], y_[-1], 'kx')

  axis_array[0].set_xlabel('X (m)')
  axis_array[0].set_ylabel('Y (m)')
  axis_array[0].axis('equal')

  plt.show()
  
def collision_found(traj, objects, walls):
  """ Return true if there is a collision with the traj and the workspace
      Arguments:
        traj (list of lists): A list of traj points - Time, X, Y, Theta (s, m, m, rad).
        objects (list of lists): A list of object states - X, Y, radius (m, m, m).
        walls (list of lists): A list of walls defined by end points - X0, Y0, X1, Y1, length (m, m, m, m, m).
      Returns:
        collision_found (boolean): True if there is a collision.
  """
  index = 0
  while index < len(traj):
    traj_point = traj[index]
    for obj in objects:
      obj_distance = generate_distance_to_object(traj_point, obj) - obj[2] - ROBOT_RADIUS
      if obj_distance < 0:
        return True
    for wall in walls:
      wall_distance = generate_distance_to_wall(traj_point, wall) - ROBOT_RADIUS
      if wall_distance < 0:
        return True
    
    index += COLLISION_INDEX_STEP_SIZE
  
  return False
  
def generate_distance_to_object(traj_point, obj):
  """ Calculate the deistance between a spherical object and a cylindrical robot.
      Argument:
        traj_point (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
        obj (list of floats): An object state X, Y, radius (m, m, m).
      Returns:
        distance (float): The distance between a traj point and an object (m).
  """
  return math.sqrt( pow(traj_point[1]-obj[0],2) + pow(traj_point[2]-obj[1],2) )
  
def generate_distance_to_wall(traj_point, wall):
  """ Calculate the deistance between a spherical object and a cylindrical robot.
      Argument:
        traj_point (list of floats): A state of Time, X, Y, Theta (s, m, m, rad).
        wall (list of floats): An wall state X0, Y0, X1, Y1, length (m, m, m, m, m).
      Returns:
        distance (float): The distance between a traj point and an object (m).
  """
  x0 = traj_point[1]
  y0 = traj_point[2]
  x1 = wall[0]
  y1 = wall[1]
  x2 = wall[2]
  y2 = wall[3]
  num = abs( (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1) )
  den = wall[4]
  
  return num/den
  
def print_traj(traj):
  """ Print a trajectory as a list of traj points.
      Arguments:
        traj (list of lists): A list of trajectory points with time, X, Y, Theta (s, m, m, rad).
  """
  print("TRAJECTORY")
  for tp in traj:
    print("traj point - time:",tp[0], "x:", tp[1], "y:", tp[2], "theta:", tp[3] )
    
def angle_diff(ang):
  """ Function to push ang within the range of -pi and pi
      Arguments:
        ang (float): An angle (rad).
      Returns:
        ang (float): The angle, but bounded within -pi and pi (rad).
  """
  while ang > math.pi:
    ang -= 2*math.pi
  while ang < -math.pi:
    ang += 2*math.pi

  return ang
  
if __name__ == '__main__':
  tp0 = [0,0,0,0]
  tp1 = [10,4,-4, -1.57]
  traj = construct_dubins_traj(tp0, tp1)
  maxR = 8
  walls = [[-maxR, maxR, maxR, maxR], [maxR, maxR, maxR, -maxR], [maxR, -maxR, -maxR, -maxR], [-maxR, -maxR, -maxR, maxR] ]
  objects = [[4, 0, 1.0], [-2, -3, 1.5]]
  plot_traj(traj, objects, walls)
