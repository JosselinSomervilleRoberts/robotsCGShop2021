import json


# ================= A CHANGER ====================== #
size_x = 600
size_y = 700
start_x = -5
start_y = -5
nx = 20
ny = 20
map_file = '../datasets/small_000_10x10_20_10.instance.json'
sol_file = '../solutions/small_000_10x10_20_10_SOLUTION_SUM124_MS25.json'
nbInterMoves = 10
# ================================================== #


s = None
data = None
data_sol = None
pos_robots = []
inc_robots = []
target_robots = []
steps = []
moveToNextStep = False
nbInterCurrent = 0
automatique = False


    
    
def getPosInitialRobots():
    global data, pos_robots, inc_robots
    pos_robots = data['starts']
    inc_robots = [[0,0] for k in range(len(pos_robots))]
    
    
def loadSteps():
    global data_sol, steps
    steps = data_sol['steps']
    
    
def nextStep():
    global steps, pos_robots, inc_robots
    
    s = steps[0]
    
    inc_robots = [[0,0] for k in range(len(pos_robots))]
    
    for indexRobot in s.keys():
        if s[indexRobot] == "N":
            inc_robots [int(indexRobot)] = [0,1]
            #pos_robots[int(indexRobot)][1] = pos_robots[int(indexRobot)][1] + 1
        elif s[indexRobot] == "S":
            inc_robots [int(indexRobot)] = [0,-1]
            #pos_robots[int(indexRobot)][1] = pos_robots[int(indexRobot)][1] - 1
        elif s[indexRobot] == "W":
            inc_robots [int(indexRobot)] = [-1,0]
            #pos_robots[int(indexRobot)][0] = pos_robots[int(indexRobot)][0] - 1
        elif s[indexRobot] == "E":
            inc_robots [int(indexRobot)] = [1,0]
            #pos_robots[int(indexRobot)][0] = pos_robots[int(indexRobot)][0] + 1
            
    steps = steps[1:]
    
    
    
def deplacerRobots():
    global pos_robots, inc_robots, nbInterMoves
    
    for i in range(len(pos_robots)):
        pos_robots[i][0] += inc_robots[i][0] / float(nbInterMoves)
        pos_robots[i][1] += inc_robots[i][1] / float(nbInterMoves)
    
    
def showRobots():
    global pos_robots, nx, ny, s, start_x, start_y
    
    i = 0
    textAlign(CENTER, CENTER) 
    for elt in pos_robots:
        i+=1
        x, y = elt[0], elt[1]
        affX = x - start_x
        affY = ny -1 - y + start_y
        fill(0,255,0,127)
        rect(affX*s, affY*s, s, s)
        fill(0,0,0)
        text(str(i), affX*s, affY*s, s, s)
        
        
        
def showTargets():
    global data, nx, ny, s, start_x, start_y
    
    i = 0
    textAlign(CENTER, CENTER) 
    for elt in data['targets']:
        i+=1
        x, y = elt[0], elt[1]
        affX = x - start_x
        affY = ny -1 - y + start_y
        fill(255,0,0)
        rect(affX*s, affY*s, s, s)
        fill(0,0,0)
        text(str(i), affX*s, affY*s, s, s)
        
        
def draw_grid():
    global data, nx, ny, s, start_x, start_y
        
    for elt in data['obstacles']:
        x, y = elt[0], elt[1]
        affX = x - start_x
        affY = ny -1 - y + start_y
        fill(0,0,0)
        rect(affX*s, affY*s, s, s)
    
    stroke(0,0,0)
    for i in range(nx+1):
        line(i*s, 0, i*s, ny*s)
        
    for j in range(ny+1):
        line(0, j*s, nx*s, j*s)
        
        
def affichage():
    background(200,200,200)
    showTargets()
    showRobots()
    draw_grid()
    
    
def setup():
    global data, nx, ny, s, size_x, size_y
    
    size(size_x, size_y)
    s = min(int(size_x/float(nx)), int(size_y/float(ny)))
    
    loadData()
    getPosInitialRobots()
    loadSteps()
    affichage()
    
    
def draw():
    global moveToNextStep, nbInterCurrent, automatique
    if moveToNextStep:
        nbInterCurrent += 1
        deplacerRobots()
        affichage()
        
        if nbInterCurrent == nbInterMoves:
            if automatique and len(steps) > 0:
                moveToNextStep = True
                nbInterCurrent = 0
                nextStep()
            else:
                moveToNextStep = False
            
        
        
        
    
    
def loadData():
    global data, data_sol, map_file, sol_file
    with open(map_file) as f:
        data = json.load(f)

    with open(sol_file) as f:
        data_sol = json.load(f)
        
        
        
def keyPressed():
    global moveToNextStep, steps, nbInterCurrent, automatique
    if keyCode == 10:
        automatique = not(automatique)
        if not(moveToNextStep) and len(steps) > 0:
            moveToNextStep = True
            nbInterCurrent = 0
            nextStep()
    if keyCode == 32:
        if len(steps) > 0 and not(moveToNextStep):
            moveToNextStep = True
            nbInterCurrent = 0
            nextStep()
