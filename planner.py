from heapq import *
import numpy
from math import *
import copy

# Directions for the theta term in the state 
E = 0
NE = 2
N = 4
NW = 6
W = 8
SW = 10
S = 12
SE = 14

# Number of distinct theta values
ANGLE_DENSITY = 16
ANGLE_UNIT = 360 / ANGLE_DENSITY

# Number of Cells Changed for a Particular Action
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

# Denotes the state of a grid cell
FREE = 0
OBSTACLE = 1
COVERED = 2
GOAL = 3

# The Number of Grid Cells a Car see's from it's Center Point
SENSOR_RAD = 5

# Dictionaries to store the Vehicle Template rotated so it doesn't get 
# recalculated every time
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
VEHICLE_X_RAD = 30
VEHICLE_Y_RAD = 15
VEHICLE_DENSITY = 3

def generate_vehicle_templates():
  template = []
  for x in range(-VEHICLE_X_RAD, VEHICLE_X_RAD+1, VEHICLE_DENSITY): 
    for y in range(-VEHICLE_Y_RAD, VEHICLE_Y_RAD+1, VEHICLE_DENSITY):
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

def are_legal_steps(state, steps, map_env):
  (r, c, theta) = state
  for step in steps:
    (dr, dc, dtheta) = step
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
    steps = self.effects(state)
    return are_legal_steps(state, steps, map_env)

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

class Halt(Action):
    def __init__(self):
        self.cost = 0

    def effects(self, state):
      return [(CONSTANT, CONSTANT, CONSTANT)]
      
    def __repr__(self):
      return "Halt"
'''
+-----------------------------------------------------------------------------+
|                                                                             |
|                                   Planner                                   |
|                                                                             |
+-----------------------------------------------------------------------------+
'''

class GraphState(object):
    def __init__(self, state, g, f, prevAction, prevGraphState, v = 0):
        self.state = state
        self.g = g
        self.f = f
        self.v = v # Only for D*
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

def heuristic(state, goals):
  maxH = 0
  for goal in goals:
    (r1, c1, _) = state
    (r2, c2, _) = goal
    delta_r = (r2 - r1) / 2
    delta_c = (c2 - c1) / 2
    distance = (delta_r * delta_r) + (delta_c * delta_c)
    maxH = max(maxH, sqrt(distance))
  return maxH

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

def A_star(start, goals, alpha, map_env, action_template):
  pq = []
  visited = set()
  heappush(pq, GraphState(start, 0, 0, None, None))
  while(len(pq) > 0):
    currState = heappop(pq)
    visited.add(currState.state)

    if(currState.state in goals):
      plan = backtrace(currState, start)
      print("Plan found:", plan)
      return plan

    
    actions = generate_successors(currState.state, map_env, action_template)

    for action in actions:
      newState = apply_action(currState.state, action.effects(currState.state))
      g = currState.g + action.get_cost()
      h = heuristic(newState, goals)
      f = g + alpha*h
      neighbor = GraphState(newState, g, f, action, currState)
      if(newState not in visited):
        heappush(pq, neighbor)
  
  print("Plan NOT found")
  return []

def computePathWithReuse(start, goals, alpha, visited, view, action_template):
  pq = []
  visited = {}
  heappush(pq, GraphState(start, 0, 0, None, None))
  while(len(pq) > 0):
    currState = heappop(pq)
    visited[currState.state] = currState

    if(currState.state in goals):
      plan = backtrace(currState, start)
      print("Plan found:", plan)
      return plan

    
    actions = generate_successors(currState.state, view, action_template)
    for action in actions:
      newState = apply_action(currState.state, action.effects(currState.state))
      g = currState.g + action.get_cost()
      h = heuristic(newState, goals)
      f = g + alpha*h
      neighbor = GraphState(newState, g, f, action, currState)
      if(newState not in visited):
        heappush(pq, neighbor)
  
  print("Plan NOT found")
  return []

def update_view(truth, view, pos):
    (r, c, _) = pos
    rad = SENSOR_RAD
    updated = []
    for dr in range(-rad, rad+1):
        for dc in range(-rad, rad+1):
            nR = r + dr
            nC = c + dc
            inRowRange = (0 <= nR) and (nR < len(truth.map))
            inColRange = (0 <= nC) and (nC < len(truth.map[0]))

            if(not inRowRange or not inColRange):
              continue

            if(truth.map[nR][nC] == OBSTACLE and view.map[nR][nC] != OBSTACLE):
                updated.append((nR, nC))
                view.map[nR][nC] = OBSTACLE
                
    return updated

def actions_to_steps(actions, start):
    steps = []
    newState = start
    for action in actions:
        steps.extend(action.effects(newState))
        newState = apply_action(newState, action.effects(newState))
    return steps

def D_star(start, goals, alpha, map_env, action_template):
  view = copy.deepcopy(map_env)

  #clear knowledge of the map
  for r in range(len(view.map)):
    for c in range(len(view.map[0])):
      view.map[r][c] = 0

  visited = {}
  actions = []
  pos = start
  count = 0
  while(True):
    current_optimal = computePathWithReuse(start, goals, alpha, visited, view,
                                           action_template)
    #check to see if path has changed and update full path based on that

    for i in range(len(current_optimal)):
      action = current_optimal[i]
      pos = apply_action(pos, action.effects(pos))
      actions.append(action)
      if(pos in goals):
        print("Number of replans required = ", count)
        return actions

      cells_updated = update_view(map_env, view, pos)
      if(cells_updated):
          future_steps = actions_to_steps(current_optimal[(i+1):], pos)
          if(not are_legal_steps(pos, future_steps, view)):
            # print("illegal:", current_optimal[(i+1):])
            break
    
    # must be the case that view was updated

    # Need to propogate inconsistent states (HARD PART)
    # for (r, c) in cells_updated:
    #   for theta in range(len(ANGLE_DENSITY)):
    #     if((r,c,theta) in visited):
    # actions.append(Halt())
    count += 1
    start = pos  

def planner_full_known(start, goals, alpha, map_env):
  action_template = generate_action_template()
  generate_vehicle_templates()
  actions = A_star(start, goals, alpha, map_env, action_template)
  plan = actions_to_steps(actions, start)
  return plan 

def planner_partial_known(start, goals, alpha, map_env):
  action_template = generate_action_template()
  generate_vehicle_templates()
  actions = D_star(start, goals, alpha, map_env, action_template)
  plan = actions_to_steps(actions, start)
  return plan

  
