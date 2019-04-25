# Autonomous Parking Simulator

from tkinter import *
from PIL import Image, ImageTk
import random
from planner import *

# state is defined as (row, col, theta)

# slight actions have two updates

# visualizer should take a sequence of updates (dr, dc, d0)


NUM_ROWS = 10
NUM_COLS = 10

E = 0
NE = 2
N = 4
NW = 6
W = 8
SW = 10
S = 12
SE = 14

'''
Actions:

sharpLeft
slightLeft
Fwd
slightRight
sharpRight

slightLeftBack
Back
slightRightBack
'''





def init(data):
    # image = Image.open("car_2inch.gif")
    data.rows = NUM_ROWS
    data.cols = NUM_COLS
    data.margin = 5 # margin around grid
    data.vehicle = [
        PhotoImage(file="car_1inch.gif"),
        PhotoImage(file="car22_5.gif"),
        PhotoImage(file="car45.gif"),
        PhotoImage(file="car67_5.gif"),
        PhotoImage(file="car90.gif"),
        PhotoImage(file="car112_5.gif"),
        PhotoImage(file="car135.gif"),
        PhotoImage(file="car157_5.gif"),
        PhotoImage(file="car180.gif"),
        PhotoImage(file="car202_5.gif"),
        PhotoImage(file="car225.gif"),
        PhotoImage(file="car247_5.gif"),
        PhotoImage(file="car270.gif"),
        PhotoImage(file="car292_5.gif"),
        PhotoImage(file="car315.gif"),   
        PhotoImage(file="car337_5.gif")
    ]
    data.timerDelay = 250

# getCellBounds from grid-demo.py
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

def mousePressed(event, data):
    pass

def isDiagonalPos(data):
    return (data.orientation % 4) == 2


# This is really just for debugging purposes
def moveForward(data, heading):
    if(data.orientation == N): takeStep(data, (-1, 0, heading))
    elif(data.orientation == NE): takeStep(data, (-1, +1, heading))
    elif(data.orientation == E): takeStep(data, (0, +1, heading))
    elif(data.orientation == SE): takeStep(data, (+1, +1, heading))
    elif(data.orientation == S): takeStep(data, (+1, 0, heading))
    elif(data.orientation == SW): takeStep(data, (+1, -1, heading))
    elif(data.orientation == W): takeStep(data, (0, -1, heading))
    elif(data.orientation == NW): takeStep(data, (-1, -1, heading))


def keyPressed(event, data):
    # if (event.keysym == "Up"):      data.direction = (-1, 0)
    # elif (event.keysym == "Down"):  data.direction = (+1, 0)
    # elif (event.keysym == "Left"):  data.direction = (0, -1)
    # elif (event.keysym == "Right"): data.direction = (0, +1)
    if (event.keysym == "q"):  moveForward(data, 2)
    elif (event.keysym == "w"):  moveForward(data, 0)
    elif (event.keysym == "e"): moveForward(data, -2)

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
    
def drawBoard(canvas, data):
    for row in range(data.rows):
        for col in range(data.cols):
            (x0, y0, x1, y1) = getCellBounds(row, col, data)
            canvas.create_rectangle(x0, y0, x1, y1, outline="red", fill="black")

def drawVehicle(canvas, data):
    (row, col) = data.pos
    (x0, y0, x1, y1) = getCellBounds(row, col, data)
    cellWidth  = (data.width - 2*data.margin) / data.cols
    cellHeight = (data.height - 2*data.margin)  / data.rows
    canvas.create_image(x0+(cellWidth/2), y0+(cellHeight/2),
                        image=data.vehicle[data.orientation])

def redrawAll(canvas, data):
    drawBoard(canvas, data)
    drawVehicle(canvas, data)

####################################
# use the run function as-is
####################################

def run(width, height, start, plan):
    def redrawAllWrapper(canvas, data):
        canvas.delete(ALL)
        canvas.create_rectangle(0, 0, data.width, data.height,
                                fill='black', width=0)
        redrawAll(canvas, data)
        canvas.update()    

    def mousePressedWrapper(event, canvas, data):
        mousePressed(event, data)
        redrawAllWrapper(canvas, data)

    def keyPressedWrapper(event, canvas, data):
        keyPressed(event, data)
        redrawAllWrapper(canvas, data)

    def timerFiredWrapper(canvas, data):
        timerFired(data)        
        redrawAllWrapper(canvas, data)
        # pause, then call timerFired again
        canvas.after(data.timerDelay, timerFiredWrapper, canvas, data)
    # Set up data and call init
    class Struct(object): pass

    (r, c, theta) = start

    data = Struct()
    data.width = width
    data.height = height
    data.pos = (r, c)
    data.orientation = theta
    data.t = 0
    data.plan = plan
    data.timerDelay = 500 # milliseconds
    root = Tk()
    root.resizable(width=False, height=False) # prevents resizing window
    init(data)
    # create the root and the canvas
    canvas = Canvas(root, width=data.width, height=data.height)
    canvas.configure(bd=0, highlightthickness=0)
    canvas.pack()
    # set up events
    root.bind("<Button-1>", lambda event:
                            mousePressedWrapper(event, canvas, data))
    root.bind("<Key>", lambda event:
                            keyPressedWrapper(event, canvas, data))
    timerFiredWrapper(canvas, data)
    # and launch the app
    root.mainloop()  # blocks until window is closed
    print("bye!")

def read_file(path):
    with open(path, "rt") as f:
        return f.read()

def parse_map(map_file):
    contents = read_file(map_file)
    nums = contents.split()
    num_rows = int(nums[0])
    num_cols = int(nums[1])
    nums = nums[2:]
    map_env = []
    for i in range(num_rows):
        col = []
        for j in range(num_cols):
            col.append(int(nums[num_cols * i + j]))
        map_env.append(col)
    return map_env



def actions_to_steps(plan, start):
    steps = []
    newState = start
    for action in plan:
        steps.extend(action.effects(newState))
        newState = apply_action(newState, action.effects(newState))
    return steps


map_env = parse_map("map1.txt")

start = (0, 0, 0)
goal = (0, 3, 0)
plan = plan(start, goal, map_env)
print(plan)

steps = actions_to_steps(plan, start)
print(steps)

# fake_plan = [(0, +1, 0), (0, +1, 0), (0, +1, 0), (+1, +1, -2)]

#visualize plan
run(800, 800, start, steps)


