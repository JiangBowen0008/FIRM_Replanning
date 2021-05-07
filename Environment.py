import math
import Robot as rbt
import random

class Environment():

  def __init__(self, size, num_objects, object_radius, robot_perception_radius, start_state, goal_state):
    self.size = size
    self.num_objects = num_objects
    self.object_radius = object_radius
    self.robot_perception_radius = robot_perception_radius
    self.start_state = start_state
    self.goal_state = goal_state

    self.walls = []
    self.objects = []
    self.visible_objects = []

    self.initialize_walls()
    # self.initialize_random_objects()
    self.initialize_benchmark_objects()

  def initialize_walls(self):
    self.walls = [[-self.size, self.size, self.size, self.size, 2*self.size], \
                  [self.size, self.size, self.size, -self.size, 2*self.size], \
                  [self.size, -self.size, -self.size, -self.size, 2*self.size], \
                  [-self.size, -self.size, -self.size, self.size, 2*self.size] ]

  def initialize_random_objects(self):
    for j in range(0, self.num_objects): 
      obj = [random.uniform(-self.size+self.object_radius*2, self.size-self.object_radius*2), \
             random.uniform(-self.size+self.object_radius*2, self.size-self.object_radius*2), \
             self.object_radius]

      while (abs(obj[0]-self.start_state[1]) < self.object_radius*2 and abs(obj[1]-self.start_state[2]) < self.object_radius*2) \
            or (abs(obj[0]-self.goal_state[1]) < self.object_radius*2 and abs(obj[1]-self.goal_state[2]) < self.object_radius*2):

        obj = [random.uniform(-self.size+self.object_radius*2, self.size-self.object_radius*2), \
               random.uniform(-self.size+self.object_radius*2, self.size-self.object_radius*2), \
               self.object_radius]
      self.objects.append(obj)
    self.update_visible_objects(self.start_state[1:])
    
  def initialize_benchmark_objects(self):
    self.objects = [[0,0,1], [-5,-5,1], [-5,-7,1], [4,-6,1], [-6,7,1], [2,7,1], [6,5,1]]
    # self.update_visible_objects(self.start_state[1:])

  def get_walls(self):
    return self.walls

  def get_all_objects(self):
    return self.objects

  def get_visible_objects(self, robot_position, time = None):
    self.update_visible_objects(robot_position, time)
    return self.visible_objects

  def get_start_state(self):
    return self.start_state

  # def get_visible_objects(self):
  #   return self.visible_objects
    
  def add_object(self, obj):
    self.objects.append(obj)    

  def update_visible_objects(self, robot_position, time = None):
    """
    Updates the list of visible objects based on whether they are in the 
    robot's perception radius or not
    """
    if time > 2:
      new_obj = [1, 1, 2]
      if not (new_obj in self.get_all_objects()):
        self.add_object(new_obj)

    objs = []
    for obj in self.objects:
      dist = math.sqrt( pow(robot_position[0]-obj[0],2) + pow(robot_position[1]-obj[1],2) )
      if dist <= self.robot_perception_radius:
        objs.append(obj)



    self.visible_objects = objs
      # if dist < self.robot_perception_radius:
      #   if obj not in self.visible_objects:
      #     self.visible_objects.append(obj)
      
      # else:
      #   if obj in self.visible_objects:
      #     self.visible_objects.remove(obj)




		

    
