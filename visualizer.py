# Autonomous Parking Simulator

from tkinter import *
from PIL import Image, ImageTk
from map_environment import *
from planner import *
from copy import copy
import time

# visualizer should take a sequence of updates (dr, dc, d0)

MAP_FILE = "40x40map2.txt"
GRID_WIDTH = 800
GRID_HEIGHT = 800

FREE = 0
OBSTACLE = 1
COVERED = 2

def update_sensors(data):
    (r, c) = data.pos 

def init(data, start, plan, map_env):
    (r, c, theta) = start
    data.rows = map_env.rows
    data.cols = map_env.cols
    data.margin = map_env.margin
    data.width = map_env.grid_width
    data.height = map_env.grid_height
    data.pos = (r, c)
    data.orientation = theta
    data.t = 0
    data.plan = plan
    data.map = copy(map_env.map)
    data.map_obj = map_env
    data.view = [[0 for c in range(data.cols)] for r in range(data.rows)]
    data.vehicle = [
        PhotoImage(file="imgs/car_1inch.gif"),
        PhotoImage(file="imgs/car22_5.gif"),
        PhotoImage(file="imgs/car45.gif"),
        PhotoImage(file="imgs/car67_5.gif"),
        PhotoImage(file="imgs/car90.gif"),
        PhotoImage(file="imgs/car112_5.gif"),
        PhotoImage(file="imgs/car135.gif"),
        PhotoImage(file="imgs/car157_5.gif"),
        PhotoImage(file="imgs/car180.gif"),
        PhotoImage(file="imgs/car202_5.gif"),
        PhotoImage(file="imgs/car225.gif"),
        PhotoImage(file="imgs/car247_5.gif"),
        PhotoImage(file="imgs/car270.gif"),
        PhotoImage(file="imgs/car292_5.gif"),
        PhotoImage(file="imgs/car315.gif"),   
        PhotoImage(file="imgs/car337_5.gif")
    ]
    # for i in data.vehicle: //TODO: look into this
    #     print(i.width(), i.height())


def getCellBounds(row, col, data):
    # aka "modelToView"
    # returns (x0, y0, x1, y1) corners/bounding box of given cell in grid
    gridWidth  = data.width - 2*data.margin
    gridHeight = data.height - 2*data.margin
    x0 = data.margin + gridWidth * col / data.cols
    x1 = data.margin + gridWidth * (col+1) / data.cols
    y0 = data.margin + gridHeight * row / data.rows
    y1 = data.margin + gridHeight * (row+1) / data.rows
    return (x0, y0, x1, y1)

def timerFired(data):
    if(data.t < len(data.plan)):
        action = data.plan[data.t]
        takeStep(data, action)
        data.t += 1

def takeStep(data, action):
    (drow, dcol, d0) = action
    (origRow, origCol) = data.pos
    (newRow, newCol) = (origRow+drow, origCol+dcol)
    if ((newRow < 0) or (newRow >= data.rows) or
        (newCol < 0) or (newCol >= data.cols)):
        print("Illegal Move")
        return

    data.pos = (newRow, newCol)
    data.orientation =  (data.orientation + d0) % 16

    (row, col) = data.pos
    (x0, y0, x1, y1) = getCellBounds(row, col, data)
    c1 = get_vehicle_coverage(x0, y0, data.orientation, data.map_obj)
    for (r,c) in c1:
        data.view[r][c] = COVERED
        data.map[r][c] = COVERED
    
    
def drawBoard(canvas, data):
    for row in range(data.rows):
        for col in range(data.cols):
            (x0, y0, x1, y1) = getCellBounds(row, col, data)            
            if(data.view[row][col] == FREE):
                canvas.create_rectangle(x0, y0, x1, y1, outline="red",
                                        fill="black")
            elif(data.view[row][col] == OBSTACLE):
                canvas.create_rectangle(x0, y0, x1, y1, outline="red",
                                        fill="yellow")
            elif(data.view[row][col] == COVERED):
                canvas.create_rectangle(x0, y0, x1, y1, outline="red",
                                        fill="blue")
            
            if(data.map[row][col] == FREE):
                canvas.create_rectangle(x0 + data.width, y0,
                                        x1 + data.width, y1, outline="red",
                                        fill="black")
            elif(data.map[row][col] == OBSTACLE):
                canvas.create_rectangle(x0 + data.width, y0,
                                        x1 + data.width, y1, outline="red",
                                        fill="yellow")
            elif(data.map[row][col] == COVERED):
                canvas.create_rectangle(x0 + data.width, y0,
                                        x1 + data.width, y1, outline="red",
                                        fill="blue")

def drawVehicle(canvas, data):
    (row, col) = data.pos
    (x0, y0, x1, y1) = getCellBounds(row, col, data)
    canvas.create_image(x0, y0,
                        image=data.vehicle[data.orientation])
    canvas.create_image(x0 + data.width,
                        y0,
                        image=data.vehicle[data.orientation])

def redrawAll(canvas, data):
    drawBoard(canvas, data)
    drawVehicle(canvas, data)


def run(start, plan, map_env):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0, (2*data.width), data.height,
                                fill='black', width=0)
        redrawAll(canvas, data)
        canvas.update()    

    def timerFiredWrapper(canvas, data):
        timerFired(data)        
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)

    # Set up data and call init
    class Struct(object): pass

    data = Struct()
    data.timerDelay = 200 # milliseconds
    root = Tk()
    root.resizable(width=False, height=False) # prevents resizing window
    init(data, start, plan, map_env)

    # create the root and the canvas
    canvas = Canvas(root, width=(2*data.width), height=data.height)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()

    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed

def actions_to_steps(plan, start):
    steps = []
    newState = start
    for action in plan:
        steps.extend(action.effects(newState))
        newState = apply_action(newState, action.effects(newState))
    return steps



if __name__ == '__main__':
    start_time = time.time()
    map_env = Map_Environment(MAP_FILE, GRID_WIDTH, GRID_HEIGHT)

    #NOTE: these coordinates must be even
    start = (8, 2, E)
    goal = (8, 34, W)
    actions = plan(start, goal, map_env)
    plan = actions_to_steps(actions, start)
    end_time = time.time()
    total_time = end_time - start_time
    print("Planner took %f seconds to compute:" % total_time)
    # #visualize plan
    run(start, plan, map_env)


