from heapq import *
import numpy
from math import *

E = 0
NE = 2
N = 4
NW = 6
W = 8
SW = 10
S = 12
SE = 14

ANGLE_DENSITY = 16
ANGLE_UNIT = 360 / ANGLE_DENSITY

SHARP_LEFT = 2
SLIGHT_LEFT = 1
STRAIGHT = 0
SLIGHT_RIGHT = -1
SHARP_RIGHT = -2

NORTH = -2
SOUTH = +2
EAST = +2
WEST = -2
CONSTANT = 0

coverage_map = {}
vehicle_template = {}

'''
+-----------------------------------------------------------------------------+
|                                                                             |
|                              Vehicle Template                               |
|                                                                             |
+-----------------------------------------------------------------------------+

Vehicle has the following dimensions:

                               72 pixels 
               |------------------------------------|

       -       +--------------------------------+
       |       |                                  )
  36   |       |                                    >
  px   |       |                                    >
       |       |                                  )
       -       +--------------------------------+

We will discretize it with the following vehicle template using (x,y) format:

'''

def generate_vehicle_templates():
  template = []
  for x in range(-31, 31, 3):
    for y in range(-15, 15, 3):
     template.append((x,y))

  for theta in range(ANGLE_DENSITY):
    r_theta = radians(-theta * ANGLE_UNIT)

    A = numpy.array([[cos(r_theta), -sin(r_theta)],
                     [sin(r_theta),  cos(r_theta)] ])

    for (dx, dy) in template:
      x = numpy.array([dx,dy])
      coords = A.dot(x)
      point = (coords[0], coords[1])
      if(theta in vehicle_template):
        vehicle_template[theta].append(point)
      else:
        vehicle_template[theta] = [point]

'''
+-----------------------------------------------------------------------------+
|                                                                             |
|                              Action Template                                |
|                                                                             |
+-----------------------------------------------------------------------------+
'''

def is_legal_point(r, c, map_env):
  
  inRowRange = (0 <= r) and (r < len(map_env.map))
  inColRange = (0 <= c) and (c < len(map_env.map[0]))

  if(not inRowRange or not inColRange):
    return False
  
  return map_env.map[r][c] == 0

def get_vehicle_coverage(x0, y0, theta, map_env):
  if (x0, y0, theta) in coverage_map:
    return coverage_map[(x0, y0, theta)]

  coverage = []
  template = vehicle_template[theta]
  for (dx, dy) in template:
    r = int((y0 + dy) / map_env.cell_height) #y
    c = int((x0 + dx) / map_env.cell_width)  #x
    coverage.append((r, c))
  
  coverage_map[(x0, y0, theta)] = coverage
  return coverage


def is_legal_position(r, c, theta, map_env):
  x0 = c * map_env.cell_width
  y0 = r * map_env.cell_height

  coverage = get_vehicle_coverage(x0, y0, theta, map_env)

  for (r, c) in coverage:
    if(not is_legal_point(r, c, map_env)):
      return False
    
  return True

def are_legal_actions(state, actions, map_env):
  (r, c, theta) = state
  for action in actions:
    (dr, dc, dtheta) = action
    theta = (theta + dtheta) % ANGLE_DENSITY
    if(theta < 0):
      theta = ANGLE_DENSITY
    (r, c) = (r+dr, c+dc)
    if(not is_legal_position(r, c, theta, map_env)):
      return False
  return True

class Action(object):
  def __init__(self):
    self.cost = 1

  def effects(self, state):
    return []

  def is_legal(self, state, map_env):
    actions = self.effects(state)
    return are_legal_actions(state, actions, map_env)

  def get_cost(self):
    return self.cost

class SlightLeft(Action):
    def __init__(self):
      self.cost = 2

    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(NORTH, CONSTANT, SLIGHT_LEFT), (NORTH, WEST, SLIGHT_LEFT)]
      elif(theta == NE):
        actions = [(NORTH, EAST, SLIGHT_LEFT), (NORTH, CONSTANT, SLIGHT_LEFT)]
      elif(theta == E):
        actions = [(CONSTANT, EAST, SLIGHT_LEFT), (NORTH, EAST, SLIGHT_LEFT)]
      elif(theta == SE):
        actions = [(SOUTH, EAST, SLIGHT_LEFT), (CONSTANT, EAST, SLIGHT_LEFT)]
      elif(theta == S):
        actions = [(SOUTH, CONSTANT, SLIGHT_LEFT), (SOUTH, EAST, SLIGHT_LEFT)]
      elif(theta == SW):
        actions = [(SOUTH, WEST, SLIGHT_LEFT), (SOUTH, CONSTANT, SLIGHT_LEFT)]
      elif(theta == W):
        actions = [(CONSTANT, WEST, SLIGHT_LEFT), (SOUTH, WEST, SLIGHT_LEFT)]
      elif(theta == NW):
        actions = [(NORTH, WEST, SLIGHT_LEFT), (CONSTANT, WEST, SLIGHT_LEFT)]
      else:
        assert(False)
      return actions
      

    def __repr__(self):
      return "Slight Left"

class SlightRight(Action):
    def __init__(self):
      self.cost = 2

    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(NORTH, CONSTANT, SLIGHT_RIGHT), (NORTH, EAST, SLIGHT_RIGHT)]
      elif(theta == NE):
        actions = [(NORTH, EAST, SLIGHT_RIGHT), (CONSTANT, EAST, SLIGHT_RIGHT)]
      elif(theta == E):
        actions = [(CONSTANT, EAST, SLIGHT_RIGHT), (SOUTH, EAST, SLIGHT_RIGHT)]
      elif(theta == SE):
        actions = [(SOUTH, EAST, SLIGHT_RIGHT), (SOUTH, CONSTANT, SLIGHT_RIGHT)]
      elif(theta == S):
        actions = [(SOUTH, CONSTANT, SLIGHT_RIGHT), (SOUTH, WEST, SLIGHT_RIGHT)]
      elif(theta == SW):
        actions = [(SOUTH, WEST, SLIGHT_RIGHT), (CONSTANT, WEST, SLIGHT_RIGHT)]
      elif(theta == W):
        actions = [(CONSTANT, WEST, SLIGHT_RIGHT), (NORTH, WEST, SLIGHT_RIGHT)]
      elif(theta == NW):
        actions = [(NORTH, WEST, SLIGHT_RIGHT), (NORTH, CONSTANT, SLIGHT_RIGHT)]
      else:
        assert(False)
      return actions

    def __repr__(self):
      return "Slight Right"

class SlightLeftBack(Action):
    def __init__(self):
      self.cost = 3

    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(SOUTH, CONSTANT, SLIGHT_RIGHT), (SOUTH, WEST, SLIGHT_RIGHT)]
      elif(theta == NE):
        actions = [(SOUTH, WEST, SLIGHT_RIGHT), (CONSTANT, WEST, SLIGHT_RIGHT)]
      elif(theta == E):
        actions = [(CONSTANT, WEST, SLIGHT_RIGHT), (NORTH, WEST, SLIGHT_RIGHT)]
      elif(theta == SE):
        actions = [(NORTH, WEST, SLIGHT_RIGHT), (NORTH, CONSTANT, SLIGHT_RIGHT)]
      elif(theta == S):
        actions = [(NORTH, CONSTANT, SLIGHT_RIGHT), (NORTH, EAST, SLIGHT_RIGHT)]
      elif(theta == SW):
        actions = [(NORTH, EAST, SLIGHT_RIGHT), (CONSTANT, EAST, SLIGHT_RIGHT)]
      elif(theta == W):
        actions = [(CONSTANT, EAST, SLIGHT_RIGHT), (SOUTH, EAST, SLIGHT_RIGHT)]
      elif(theta == NW):
        actions = [(SOUTH, EAST, SLIGHT_RIGHT), (SOUTH, CONSTANT, SLIGHT_RIGHT)]
      else:
        assert(False)
      return actions
    
    def __repr__(self):
      return "Slight Left Back"

class SlightRightBack(Action):
    def __init__(self):
      self.cost = 3

    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(SOUTH, CONSTANT, SLIGHT_LEFT), (SOUTH, EAST, SLIGHT_LEFT)]
      elif(theta == NE):
        actions = [(SOUTH, WEST, SLIGHT_LEFT), (SOUTH, CONSTANT, SLIGHT_LEFT)]
      elif(theta == E):
        actions = [(CONSTANT, WEST, SLIGHT_LEFT), (SOUTH, WEST, SLIGHT_LEFT)]
      elif(theta == SE):
        actions = [(NORTH, WEST, SLIGHT_LEFT), (CONSTANT, WEST, SLIGHT_LEFT)]
      elif(theta == S):
        actions = [(NORTH, CONSTANT, SLIGHT_LEFT), (NORTH, WEST, SLIGHT_LEFT)]
      elif(theta == SW):
        actions = [(NORTH, EAST, SLIGHT_LEFT), (NORTH, CONSTANT, SLIGHT_LEFT)]
      elif(theta == W):
        actions = [(CONSTANT, EAST, SLIGHT_LEFT), (NORTH, EAST, SLIGHT_LEFT)]
      elif(theta == NW):
        actions = [(SOUTH, EAST, SLIGHT_LEFT), (CONSTANT, EAST, SLIGHT_LEFT)]
      else:
        assert(False)
      return actions

    def __repr__(self):
      return "Slight Right Back"    

class SharpLeft(Action):
    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(NORTH, WEST, SHARP_LEFT)]
      elif(theta == NE):
        actions = [(NORTH, CONSTANT, SHARP_LEFT)]
      elif(theta == E):
        actions = [(NORTH, EAST, SHARP_LEFT)]
      elif(theta == SE):
        actions = [(CONSTANT, EAST, SHARP_LEFT)]
      elif(theta == S):
        actions = [(SOUTH, EAST, SHARP_LEFT)]
      elif(theta == SW):
        actions = [(SOUTH, CONSTANT, SHARP_LEFT)]
      elif(theta == W):
        actions = [(SOUTH, WEST, SHARP_LEFT)]
      elif(theta == NW):
        actions = [(CONSTANT, WEST, SHARP_LEFT)]
      else:
        assert(False)
      return actions
      
    def __repr__(self):
      return "Sharp Left"
        
class SharpRight(Action):
    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(NORTH, EAST, SHARP_RIGHT)]
      elif(theta == NE):
        actions = [(CONSTANT, EAST, SHARP_RIGHT)]
      elif(theta == E):
        actions = [(SOUTH, EAST, SHARP_RIGHT)]
      elif(theta == SE):
        actions = [(SOUTH, CONSTANT, SHARP_RIGHT)]
      elif(theta == S):
        actions = [(SOUTH, WEST, SHARP_RIGHT)]
      elif(theta == SW):
        actions = [(CONSTANT, WEST, SHARP_RIGHT)]
      elif(theta == W):
        actions = [(NORTH, WEST, SHARP_RIGHT)]
      elif(theta == NW):
        actions = [(NORTH, CONSTANT, SHARP_RIGHT)]
      else:
        assert(False)
      return actions
      
    def __repr__(self):
      return "Sharp Right"

class Forward(Action):
    def __init__(self):
      self.cost = 1

    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(NORTH, CONSTANT, STRAIGHT)]
      elif(theta == NE):
        actions = [(NORTH, EAST, STRAIGHT)]
      elif(theta == E):
        actions = [(CONSTANT, EAST, STRAIGHT)]
      elif(theta == SE):
        actions = [(SOUTH, EAST, STRAIGHT)]
      elif(theta == S):
        actions = [(SOUTH, CONSTANT, STRAIGHT)]
      elif(theta == SW):
        actions = [(SOUTH, WEST, STRAIGHT)]
      elif(theta == W):
        actions = [(CONSTANT, WEST, STRAIGHT)]
      elif(theta == NW):
        actions = [(NORTH, WEST, STRAIGHT)]
      else:
        assert(False)
      return actions
      
    def __repr__(self):
      return "Forward"

class Backward(Action):
    def __init__(self):
        self.cost = 2

    def effects(self, state):
      (_, _, theta) = state
      if(theta == N):
        actions = [(SOUTH, CONSTANT, STRAIGHT)]
      elif(theta == NE):
        actions = [(SOUTH, WEST, STRAIGHT)]
      elif(theta == E):
        actions = [(CONSTANT, WEST, STRAIGHT)]
      elif(theta == SE):
        actions = [(NORTH, WEST, STRAIGHT)]
      elif(theta == S):
        actions = [(NORTH, CONSTANT, STRAIGHT)]
      elif(theta == SW):
        actions = [(NORTH, EAST, STRAIGHT)]
      elif(theta == W):
        actions = [(CONSTANT, EAST, STRAIGHT)]
      elif(theta == NW):
        actions = [(SOUTH, EAST, STRAIGHT)]
      else:
        assert(False)
      return actions
      
    def __repr__(self):
      return "Backward"

'''
+-----------------------------------------------------------------------------+
|                                                                             |
|                                   Planner                                   |
|                                                                             |
+-----------------------------------------------------------------------------+
'''

class GraphState(object):
    def __init__(self, state, g, f, prevAction, prevGraphState):
        self.state = state
        self.g = g
        self.f = f
        self.prevAction = prevAction
        self.prevGraphState = prevGraphState

    def __hash__(self):
        return hash(self.state)

    def __eq__(self, other):
        return (isinstance(other, GraphState) and (self.state == other.state))

    def __lt__(self, other):
      return self.f < other.f

    def __repr__(self):
      return "GraphState"

def heuristic(state, goal):
  (r1, c1, _) = state
  (r2, c2, _) = goal
  delta_r = (r2 - r1) / 2
  delta_c = (c2 - c1) / 2
  distance = (delta_r * delta_r) + (delta_c * delta_c)
  return sqrt(distance)



def generate_action_template():
  action_template = []

  action_template.append(SharpLeft())
  action_template.append(SlightLeft())
  action_template.append(Forward())
  action_template.append(SlightRight())
  action_template.append(SharpRight())

  action_template.append(SlightLeftBack())
  action_template.append(Backward())
  action_template.append(SlightRightBack())

  return action_template

def generate_successors(state, map_env, action_template):
  successors = []
  for action in action_template:
    if(action.is_legal(state, map_env)):
      successors.append(action)
  
  return successors

def apply_action(state, action):
  (r, c, theta) = state
  for move in action:
    (dr, dc, dtheta) = move
    new_theta = (theta + dtheta) % ANGLE_DENSITY
    if(new_theta < 0):
      new_theta = ANGLE_DENSITY
    (r, c, theta) = (r+dr, c+dc, new_theta)
  return (r, c, theta)

def backtrace(graphState, start):
  actions = []
  while(graphState.state != start):
    actions.append(graphState.prevAction)
    graphState = graphState.prevGraphState
  actions.reverse()    
  return actions

def A_star(start, goal, map_env, action_template):
  pq = []
  visited = set()
  heappush(pq, GraphState(start, 0, 0, None, None))
  while(len(pq) > 0):
    currState = heappop(pq)
    visited.add(currState)

    if(currState.state == goal):
      plan = backtrace(currState, start)
      print("Plan found:", plan)
      return plan

    
    actions = generate_successors(currState.state, map_env, action_template)
    for action in actions:
      newState = apply_action(currState.state, action.effects(currState.state))
      g = currState.g + action.get_cost()
      h = heuristic(newState, goal)
      f = g + 5*h
      neighbor = GraphState(newState, g, f, action, currState)
      if(neighbor not in visited):
        heappush(pq, neighbor)
  
  print("Plan NOT found")
  return []


'''
start = (x, y, 0)
goal = (x, y, 0)
map will be a 2D array of numbers
  0 = Free
  1 = Obstacle
  TODO: add vehicle in different orientations as obstacles later

This planner will return a sequence of actions in an array. Each action has 
the format (dx, dy, d0)
'''
def plan(start, goal, map_env):
  action_template = generate_action_template()
  generate_vehicle_templates()
  return A_star(start, goal, map_env, action_template)
