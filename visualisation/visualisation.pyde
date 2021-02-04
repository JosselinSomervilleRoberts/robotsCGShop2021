import json


# ================= A CHANGER ====================== #
size_x = 880
size_y = 980
start_x = -7
start_y = -7
nx = 24
ny = 24
map_file = '../datasets/small_free_001_10x10_40_40.instance.json'
sol_file = '../solutions/small_free_001_10x10_40_40.instance/2021_02_04_19_21_19_1___g.json'
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

makespan = None
total_distance = None
distance = 0
actual_makespan = 0
inc_nbIntermoves = 0
targetsOnTop = False
transparence = True
optimise = False


    
    
def getPosInitialRobots():
    global data, pos_robots, inc_robots
    pos_robots = [[elt[0], elt[1]] for elt in data['starts']]
    inc_robots = [[0,0] for k in range(len(pos_robots))]
    
    
def loadSteps():
    global data_sol, steps, makespan, total_distance, actual_makespan, distance
    steps = data_sol['steps']
    makespan = len(steps)
    distance = 0
    actual_makespan = 0
    
    total_distance = 0
    for s in steps:
        total_distance += len(s.keys())
    
    
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
    global pos_robots, inc_robots, nbInterMoves, distance, actual_makespan
    
    actual_makespan += 1/float(nbInterMoves)
    
    for i in range(len(pos_robots)):
        pos_robots[i][0] += inc_robots[i][0] / float(nbInterMoves)
        pos_robots[i][1] += inc_robots[i][1] / float(nbInterMoves)
        
        if(inc_robots[i] != [0,0]): 
            distance += 1/float(nbInterMoves)
    
    
def showRobots():
    global pos_robots, nx, ny, s, start_x, start_y, data, transparence, optimise
    
    i = -1
    textSize(int(s/2.0))
    textAlign(CENTER, CENTER)
    stroke(0,0,0)
    strokeWeight(1)
    for elt in pos_robots:
        i+=1
        x, y = elt[0], elt[1]
        affX = x - start_x
        affY = ny -1 - y + start_y
        
        if optimise:
            fill(0,255,0)
        else:
            fill(0,255,0,255-127*transparence)
        if i==4:
            fill(0,0,0)
        
        if data['targets'][i][0] == int(round(x,0)) and data['targets'][i][1] == int(round(y,0)):
            fill(255,0,255)
        rect(affX*s, affY*s, s, s)
        
        if not(optimise):
            fill(0,0,0)
            text(str(i), affX*s, affY*s, s, s)
        
        
        
def showTargets():
    global data, nx, ny, s, start_x, start_y, transparence, pos_robots, optimise
    
    i = -1
    textSize(int(s/2.0))
    textAlign(CENTER, CENTER)
    stroke(0,0,0)
    strokeWeight(1)
    for elt in data['targets']:
        i+=1
        x, y = elt[0], elt[1]
        affX = x - start_x
        affY = ny -1 - y + start_y
        
        if optimise:
            fill(255,0,0)
        else:
            fill(255,0,0, 255 - 127*transparence)
        if i==4:
            fill(50,50,50)
        
        if int(round(pos_robots[i][0],0)) == x and int(round(pos_robots[i][1],0)) == y:
            fill(255,0,255)
        rect(affX*s, affY*s, s, s)
        
        if not(optimise):
            fill(0,0,0)
            text(str(i), affX*s, affY*s, s, s)
        
        
def draw_grid():
    global data, nx, ny, s, start_x, start_y, optimise
        
    for elt in data['obstacles']:
        x, y = elt[0], elt[1]
        affX = x - start_x
        affY = ny -1 - y + start_y
        fill(0,0,0)
        rect(affX*s, affY*s, s, s)
    
    if not(optimise):
        stroke(0,0,0)
        strokeWeight(1)
        for i in range(nx+1):
            line(i*s, 0, i*s, ny*s)
            
        for j in range(ny+1):
            line(0, j*s, nx*s, j*s)
        
        
def affichage():
    global size_x, size_y, makespan, steps, total_distance, distance, pos_robots, actual_makespan, targetsOnTop, optimise
    background(200,200,200)
    
    if targetsOnTop:
        showRobots()
        if not(optimise):
            showTargets()
        draw_grid()
    else:
        showTargets()
        if not(optimise):
            showRobots()
        draw_grid()
    
    fill(0,0,0)
    rect(0, size_y - 100, size_x, 100)
    
    t = int(size_x/16.0)
    fill(255,100,100)
    stroke(200,50,50)
    strokeWeight(3)
    
    rect(t, size_y - 80, 4*t, 60)
    rect(6*t, size_y - 80, 4*t, 60)
    rect(11*t, size_y - 80, 4*t, 60)
    
    fill(0,0,0)
    textAlign(CENTER, CENTER)
    textSize(int(size_x/45.0))
    text("Nombre robots", t, size_y - 80, 4*t, 30)
    text(str(len(pos_robots)), t, size_y - 50, 4*t, 25)
    text("Makespan", 6*t, size_y - 80, 4*t, 30)
    text(str(round(actual_makespan,1)) + " / " + str(makespan), 6*t, size_y - 50, 4*t, 25)
    text("Distance", 11*t, size_y - 80, 4*t, 30)
    text(str(ceil(distance)) + " / " + str(total_distance), 11*t, size_y - 50, 4*t, 25)
    
    
    
def setup():
    global data, nx, ny, s, size_x, size_y, optimise
    
    size(size_x, size_y)
    s = min(int(size_x/float(nx)), int(size_y-100/float(ny)))
    
    if s<=6:
        optimise = True
        
    loadData()
    getPosInitialRobots()
    loadSteps()
    affichage()
    
    
def draw():
    global moveToNextStep, nbInterCurrent, automatique, nbInterMoves, inc_nbIntermoves
    if moveToNextStep:
        nbInterCurrent += 1
        deplacerRobots()
        affichage()
        
        if nbInterCurrent == nbInterMoves:
            nbInterMoves += inc_nbIntermoves
            nbInterMoves = max(1, nbInterMoves)
            inc_nbIntermoves = 0
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
    global moveToNextStep, steps, nbInterCurrent, automatique, inc_nbIntermoves, nbInterMoves, targetsOnTop, transparence
    print(keyCode)
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
            
    if keyCode == 82: #r
        if len(steps) == 0 or not(moveToNextStep):
            getPosInitialRobots()
            loadSteps()
            affichage()
            automatique = False
            
    if keyCode == 65: #a
        if not(moveToNextStep):
            nbInterMoves -= 1
            nbInterMoves = max(1, nbInterMoves)
        else:
            inc_nbIntermoves -= 1
    if keyCode == 83: #s
        if not(moveToNextStep):
            nbInterMoves += 1
        else:
            inc_nbIntermoves += 1
            
    if keyCode == 90: #Z
        targetsOnTop = not(targetsOnTop)
        if not(moveToNextStep):
            affichage()
            
    if keyCode == 69: #E
        transparence = not(transparence)
        if not(moveToNextStep):
            affichage()
