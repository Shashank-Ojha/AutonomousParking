from heapq import *


'''
+-----------------------------------------------------------------------------+
|                                                                             |
|                              Action Template                                |
|                                                                             |
+-----------------------------------------------------------------------------+
'''

E = 0
NE = 2
N = 4
NW = 6
W = 8
SW = 10
S = 12
SE = 14

ANGLE_DENSITY = 16

SHARP_LEFT = 2
SLIGHT_LEFT = 1
STRAIGHT = 0
SLIGHT_RIGHT = -1
SHARP_RIGHT = 2

NORTH = -1
SOUTH = +1
EAST = +1
WEST = -1
CONSTANT = 0

def is_legal_position(r, c, map_env):
  inRowRange = (0 <= r) and (r < len(map_env))
  inColRange = (0 <= c) and (c < len(map_env[0]))

  if(not inRowRange or not inColRange):
    return False
  
  return map_env[r][c] == 0


def are_legal_actions(state, actions, map_env):
  (r, c, _) = state
  for action in actions:
    (dr, dc, _) = action
    (r, c) = (r+dr, c+dc)
    if(not is_legal_position(r, c, map_env)):
      return False
  
  return True

class slightLeft(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)

    def __repr__(self):
      return "slightLeft"

class slightRight(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)

    def __repr__(self):
      return "slightRight"

class slightLeftBack(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)
    
    def __repr__(self):
      return "slightLeftBack"

class slightRightBack(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)

    def __repr__(self):
      return "slightRightBack"    

class sharpLeft(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)
    
    def __repr__(self):
      return "sharpLeft"
        
class sharpRight(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)

    def __repr__(self):
      return "sharpRight"

class forward(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)

    def __repr__(self):
      return "forward"

class backward(object):
    def __init__(self):
        pass

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
      
    def is_legal(self, state, map_env):
      actions = self.effects(state)
      return are_legal_actions(state, actions, map_env)

    def __repr__(self):
      return "backward"

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

  
def generate_action_template():
  action_template = []

  action_template.append(sharpLeft())
  action_template.append(slightLeft())
  action_template.append(forward())
  action_template.append(slightRight())
  action_template.append(sharpRight())

  action_template.append(slightLeftBack())
  action_template.append(backward())
  action_template.append(slightRightBack())

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
    #start with just A*
  pq = []
  visited = set()
  heappush(pq, GraphState(start, 0, 0, None, None))
  while(len(pq) > 0):
    currState = heappop(pq)
    visited.add(currState)

    if(currState.state == goal):
      print("Goal found")
      plan = backtrace(currState, start)
      print("plan:", plan)
      return plan

    
    actions = generate_successors(currState.state, map_env, action_template)
    for action in actions:
      newState = apply_action(currState.state, action.effects(currState.state))
      f = currState.g + 1
      neighbor = GraphState(newState, currState.g + 1, f, action, currState)
      if(neighbor not in visited):
        heappush(pq, neighbor)
  
  print("Goal NOT found")
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
  print("start", start)
  action_template = generate_action_template()
  return A_star(start, goal, map_env, action_template)
