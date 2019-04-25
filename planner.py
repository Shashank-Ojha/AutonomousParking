'''
+-----------------------------------------------------------------------------+
|                                                                             |
|                              Action Template                                |
|                                                                             |
+-----------------------------------------------------------------------------+
'''

class slightLeft(object):
    def __init__(self):
        pass

    def is_legal(self, state):

        

    def effects(self, state):
        

'''
+-----------------------------------------------------------------------------+
|                                                                             |
|                                   Planner                                   |
|                                                                             |
+-----------------------------------------------------------------------------+
'''

def generate_action_template()
  action_template = []
  action_template.append()

def generate_successors(state):
  
  '''
  '''

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
def plan(start, goal, map):
  #start with just A*
  '''
  init empty pq
  init empty visited set
  init state with cost 0 and pos = start
  while(!pq_empty):
    s = pq_remove
    insert s in visited //double check this line for D*
    if(s.pos == goal):
      backtrace and return plan
    
    succ = generate_successors
    for(n in succ):
      if(n not visited):
        f = g + h
        insert into pq
  '''
  pass