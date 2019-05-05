# Autonomous Parking Simulator

import sys
from tkinter import *
from PIL import Image, ImageTk
from map_environment import *
from planner import *
from copy import copy
import time

# visualizer should take a sequence of updates (dr, dc, d0)

MAP_FILE = "40x40map4.txt"
FULL = 0
PARTIAL = 1

#NOTE: these coordinates must be even
start_config = (8, 2, E)
goal_configs = [(8, 34, E)]
# goal_configs = [(8, 34, E), (20, 26, E)]
model_type = FULL
alpha = 1


GRID_WIDTH = 800
GRID_HEIGHT = 800

def update_sensors(data):
    if(model_type == FULL): return

    (r, c) = data.pos
    rad = SENSOR_RAD
    for dr in range(-rad, rad+1):
        for dc in range(-rad, rad+1):
            newR = r + dr
            newC = c + dc
            inRowRange = (0 <= newR) and (newR < len(data.map))
            inColRange = (0 <= newC) and (newC < len(data.map[0]))

            if(inRowRange and inColRange and data.map[newR][newC] == OBSTACLE):
                data.view[newR][newC] = OBSTACLE

def init(data, start, plan, goals, map_env):
    (r, c, theta) = start
    data.timerDelay = 200 # milliseconds
    data.divider_size = 80
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
    for r in range(data.rows):
        for c in range(data.cols):
            if(data.map[r][c] == COVERED):
                data.map[r][c] = FREE
    
    data.map_obj = map_env
    if(model_type == PARTIAL):
        data.view = [[0 for c in range(data.cols)] for r in range(data.rows)]
    else:
        data.view = copy(map_env.map)
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

    # mark goals
    for goal in goals:
        (r_g, c_g, theta_g) = goal
        x0 = c_g * map_env.cell_width
        y0 = r_g * map_env.cell_height
        c1 = get_vehicle_coverage(x0, y0, theta_g, data.map_obj)
        for (r,c) in c1:
            data.view[r][c] = GOAL
            data.map[r][c] = GOAL

    # for i in data.vehicle: //TODO: look into this
    #     print(i.width(), i.height())

# returns (x0, y0, x1, y1) corners/bounding box of given cell in grid
def getCellBounds(row, col, data):
    gridWidth  = data.width - 2*data.margin
    gridHeight = data.height - 2*data.margin
    x0 = data.margin + gridWidth * col / data.cols
    x1 = data.margin + gridWidth * (col+1) / data.cols
    y0 = data.margin + gridHeight * row / data.rows
    y1 = data.margin + gridHeight * (row+1) / data.rows
    return (x0, y0, x1, y1)

def keyPressed(event, data, start, plan, goal, map_env):
    if (event.keysym == "r"): #reset simulation
        init(data, start, plan, goal, map_env)

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
    
    # Update Tracks as you go
    x0 = newCol * map_env.cell_width
    y0 = newRow * map_env.cell_height

    c1 = get_vehicle_coverage(x0, y0, data.orientation, data.map_obj)
    for (r,c) in c1:
        data.view[r][c] = COVERED
        data.map[r][c] = COVERED

    update_sensors(data)
       
def drawBoard(canvas, data):
    # Draw Divider
    canvas.create_rectangle(data.width, 0,
                            data.width + data.divider_size, data.height,
                            outline="blue",
                            fill="black")

    # Draw both the Driver View and the Full Model Boards
    for row in range(data.rows):
        for col in range(data.cols):
            (x0, y0, x1, y1) = getCellBounds(row, col, data)            
            if(data.view[row][col] == FREE):
                canvas.create_rectangle(x0, y0, x1, y1, outline="blue",
                                        fill="black")
            elif(data.view[row][col] == OBSTACLE):
                canvas.create_rectangle(x0, y0, x1, y1, outline="blue",
                                        fill="blue")
            elif(data.view[row][col] == COVERED):
                canvas.create_rectangle(x0, y0, x1, y1, outline="blue",
                                        fill="gray")
            elif(data.view[row][col] == GOAL):
                canvas.create_rectangle(x0, y0, x1, y1, outline="blue",
                                        fill="red")
            
            if(data.map[row][col] == FREE):
                canvas.create_rectangle(x0 + data.width + data.divider_size, y0,
                                        x1 + data.width + data.divider_size, y1,
                                        outline="blue",
                                        fill="black")
            elif(data.map[row][col] == OBSTACLE):
                canvas.create_rectangle(x0 + data.width + data.divider_size, y0,
                                        x1 + data.width + data.divider_size, y1,
                                        outline="blue",
                                        fill="blue")
            elif(data.map[row][col] == COVERED):
                canvas.create_rectangle(x0 + data.width + data.divider_size, y0,
                                        x1 + data.width + data.divider_size, y1,
                                        outline="blue",
                                        fill="gray")
            elif(data.map[row][col] == GOAL):
                canvas.create_rectangle(x0 + data.width + data.divider_size, y0,
                                        x1 + data.width + data.divider_size, y1,
                                        outline="blue",
                                        fill="red")

def drawVehicle(canvas, data):
    (row, col) = data.pos
    (x0, y0, x1, y1) = getCellBounds(row, col, data)
    canvas.create_image(x0, y0,
                        image=data.vehicle[data.orientation])
    canvas.create_image(x0 + data.width + data.divider_size,
                        y0,
                        image=data.vehicle[data.orientation])

def drawLabels(canvas, data):
    canvas.create_text(data.width/2, 740,
                       fill="green", text="Driver View", font="Lato 40")
    canvas.create_text((1.5)*data.width + data.divider_size, 740,
                       fill="green", text="Full Model", font="Lato 40")

def redrawAll(canvas, data):
    drawBoard(canvas, data)
    drawVehicle(canvas, data)
    drawLabels(canvas, data)

def setup_canvas(root, data):
    canvas = Canvas(root,
                    width=(2*data.width) + data.divider_size,
                    height=data.height)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()
    return canvas

def visualize(start, plan, goal, map_env):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0,
                                (2*data.width) + data.divider_size,
                                data.height,
                                fill='black', width=0)
        redrawAll(canvas, data)
        canvas.update()    
    
    def keyPressedWrapper(event, canvas, data, start, plan, goal, map_env):
        keyPressed(event, data, start, plan, goal, map_env)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas, data):
        timerFired(data)        
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)

    # Set up data and call init
    class Visualizer(object): pass

    data = Visualizer()
    root = Tk()
    root.resizable(width=False, height=False) # prevents resizing window
    root.title("Autonomous Vehicle Parking Planner Simulator")
    init(data, start, plan, goal, map_env)

    # create the canvas
    canvas = setup_canvas(root, data)

    root.bind("<Key>", lambda event:
         keyPressedWrapper(event, canvas, data, start, plan, goal, map_env))
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed

def run_planner():
    start_time = time.time()
    if(model_type == FULL):
        plan = planner_full_known(start_config, goal_configs, alpha, map_env)
    else:
        plan = planner_partial_known(start_config, goal_configs, alpha, map_env)

    end_time = time.time()
    total_time = end_time - start_time

    print("Planner took %f seconds to compute:" % total_time)

    return plan

def usage():
    print("USAGE:")
    print("\t python3 visualizer.py  [alpha] --full")
    print("\t python3 visualizer.py  [alpha] --partial")

def parse_arguments():
    global model_type
    global alpha
    try:
        alpha = float(sys.argv[1])
        mtype = sys.argv[2][2:]
        if(mtype == "partial"):
            model_type = PARTIAL
        elif(mtype == "full"):
            model_type = FULL
        else:
            usage()
            sys.exit(1)
    except:
        usage()
        sys.exit(1)

if __name__ == '__main__':
    parse_arguments()
    map_env = Map_Environment(MAP_FILE, GRID_WIDTH, GRID_HEIGHT)
    plan = run_planner()

    # visualize plan
    visualize(start_config, plan, goal_configs, map_env)


