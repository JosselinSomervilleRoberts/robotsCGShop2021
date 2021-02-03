# -*- coding: utf-8 -*-
"""
Created on Wed Feb  3 02:00:35 2021

@author: josse
"""

# -*- coding: utf-8 -*-
"""
Created on Fri Dec 18 21:38:35 2020
@author: olivi
"""
import numpy as np
import time
from cgshop2021_pyutils import Instance, InstanceDatabase
from cgshop2021_pyutils import Solution, SolutionStep, SolutionZipWriter, Direction
from cgshop2021_pyutils import ZipSolutionIterator, validate, ZipReaderError, InvalidSolutionError, SolutionEncodingError
from copy import copy, deepcopy
import random
import os
import zipfile
import shutil
import glob
from map import Map
from datetime import datetime


INFINITY = 99
OBS = -2
NORD = (0,1)
N = 0
SUD = (0,-1)
S = 1
EST= (1,0)
E = 2
OUEST = (-1,0)
O = 3
ZERO = (0,0)
Z = 4
directions = [NORD,SUD,EST,OUEST,ZERO]
directions_utils = [Direction.NORTH,Direction.SOUTH,Direction.EAST,Direction.WEST,Direction.WAIT]

def index(d):
    if d==NORD:
        return N
    if d==SUD:
        return S
    if d==EST:
        return E
    if d==OUEST:
        return O
    if d==ZERO:
        return Z



def dist(t1,t2):
    '''La distance de la valeur absolue entre deux tuples'''
    return abs(t1[0]-t2[0])+abs(t1[1]-t2[1])


class Pilote :
    
    
    def __init__(self, index, nx, ny, cible, depart, obstacles=None):
        self.depart = depart # case de de depart
        self.p = depart #La position actuelle du robot
        self.index = index # index du robot dans la liste pilote
        self.cible = cible #La ou le robot doit aller

        self.dernier_pas = ZERO
        self.lastDir = Direction.WAIT
        self.carte = Map(nx, ny)
        if not(obstacles is None):
            self.carte.ajouterObstaclesPermanents(obstacles)
        self.carte.setCible(cible)
        
        self.r = 0.3
        self.a = 0.2
        self.d = 4
        self.poisson = 0.2
        self.porteeRepulsion = 2
        
    def reset(self):
        self.p =self.depart
        self.dernier_pas =ZERO
        self.carte.resetMap()
        #self.calculerChemin()
        
        
    def ajouterObstacles(self, liste_obs, recalculer=True):
        self.carte.ajouterObstacles(liste_obs)
        if recalculer:
            self.carte.bfs_nouveaux_obstacles(liste_obs)
            
            
    def enleverObstacles(self, liste_obs, recalculer=True):
        self.carte.enleverObstacles(liste_obs)
        if recalculer:
            raise NotImplementedError
        
    
    def distance(self):
        '''En combien de pas le robot peut-il arriver à destination'''
        return self.carte.map[self.p[0]][self.p[1]]
    
    
    def params(self,r,a,d, poisson):
        self.r = r
        self.a = a
        self.d = d
        self.poisson = poisson
        
    def move(self, inc):
        self.p = (self.p[0]+inc[0],self.p[1]+inc[1])
        self.lastDir = directions_utils[index(inc)]
        
    def pas_probabiliste(self,cases_prises=[]):
        REPULSION = self.r
        ALEATOIRE = self.a
        DETERMINISTE = self.d
        POISSON = self.poisson
        '''Dans la stratégie *banc de poissons, on demande au robot de faire un pas
        Il renvoie le pas choisi en format tuple'''
        p = self.p
        cases_prises_relat = [ (r[0]-p[0],r[1]-p[1], r[2]) for r in cases_prises]
        poids_direction = [ 0 for i in range(5)]
        
        
        decisions_possibles = [ (elt[0]-p[0],elt[1]-p[1]) for elt in self.carte.getVoisinsXY(p)] + [(0,0)]

        
        sameDir = []
        
        cases_prises_proches = []
        for r in cases_prises_relat :
            if r[0] !=  0 or r[1] != 0 :
                x = r[0]
                y = r[1]
                di = r[2]
                ajouterCase = True
                
                if x==0 and y==1 and di==Direction.NORTH:
                    ajouterCase = False
                elif x==0 and y==-1 and di==Direction.SOUTH:
                    ajouterCase = False
                elif x==-1 and y==0 and di==Direction.WEST:
                    ajouterCase = False
                elif x==1 and y==0 and di==Direction.EAST:
                    ajouterCase = False
                
                d = abs(r[0])+abs(r[1])
                if ajouterCase:
                    if d < self.porteeRepulsion+1:
                        cases_prises_proches.append((x,y))
                else:
                    sameDir.append((x,y))
        
        for c in cases_prises_proches :
            for d in decisions_possibles :
                dis = dist(c,d)
                if dis > 0:
                    poids_direction[index(d)]-=REPULSION / dis
        
        for d in decisions_possibles :
            d_abs = (p[0]+d[0],p[1]+d[1])
            delta = self.carte.getValue(p) - self.carte.getValue(d_abs)
            
            if delta > 0: delta *= DETERMINISTE
            
            if d in sameDir:
                delta += POISSON
            
            poids_direction[index(d)] += delta
        
        interdits = []
        for i in range(len(poids_direction)) :
            a = directions[i]
            if (a not in decisions_possibles) or (a in cases_prises_proches) :
                poids_direction[i]=0
                interdits.append(i)
        
        mini = min(poids_direction)
        for i in range(len(poids_direction)):
            if i not in interdits :
                poids_direction[i] += ALEATOIRE -mini

        pas = random.choices(directions, weights=poids_direction, k=1)[0]
        self.move(pas)
        return self.lastDir
        print("Erreur !")
        


"""
def calculPriorites(pilotes):
    pasEncoreArrives = [(p.cible, p.index) for p in pilotes]
    groupes = []
    
    while len(pasEncoreArrives) > 0:
        groupes.append([])
        onlyCasesArrives = [elt[0] for elt in pasEncoreArrives]
        [pilotes[elt[1]].ajouterObstacles(onlyCasesArrives) for elt in pasEncoreArrives]
        toRemove = []
        for elt in pasEncoreArrives:
            (case_arrivee, index) = elt
            if pilotes[index].T[(0,0)] >= 0: #pilotes[index].p
                groupes[-1].append(index)
                toRemove.append(elt)
                
        for elt in toRemove:
            pasEncoreArrives.remove(elt)
            
        [pilotes[elt[1]].enleverObstacles([elt[0] for elt in toRemove], recalculer=False) for elt in pasEncoreArrives]
            
    return groupes
"""



def calculPriorites2(pilotes):
    pasEncoreArrives = [p.index for p in pilotes]
    dejaArrives = []
    obstacles = []
    groupes = []
    distance = 0
    
    while len(pasEncoreArrives) > 0:
        groupes.append([])
        #[pilotes[index].enleverObstacles(obstacles, recalculer=False) for index in pasEncoreArrives]
        [pilotes[index].carte.resetMap() for index in pasEncoreArrives]
        obstacles = [pilotes[index].cible for index in pasEncoreArrives] + [pilotes[index].p for index in dejaArrives]
        [pilotes[index].ajouterObstacles(obstacles, recalculer=True) for index in pasEncoreArrives]
        ajout = False
        
        for index in copy(pasEncoreArrives):
            if pilotes[index].carte.getValue(pilotes[index].p) >= 0: # s'il existe un chemin
                distance += pilotes[index].carte.getValue(pilotes[index].p)
                groupes[-1].append(index)
                pasEncoreArrives.remove(index)
                dejaArrives.append(index)
                ajout = True
                
        if not(ajout):
            print("IMPOSSIBLE")
            print(pasEncoreArrives)
            print("\n")
            print(groupes)
            return None
                
        
    print("Distance = ", distance)
    return groupes




def getBoxes(pilotes, xStart, yStart, widthX, widthY):
    boxs = [[[xStart,yStart,widthX,widthY, pilotes]]]
    liste_directions = [Direction.SOUTH,Direction.WEST, Direction.NORTH,Direction.EAST]
    continuer = True
    
    i = -1
    while continuer:
        i += 1
        continuer = False
        boxs.append([])
        boxs.append([])
        for b in boxs[i]:
            if b[2] > 1 and b[3] > 1 and len(b[4]) > 1:
                continuer = True
                for x in range(2):
                    startX = b[0] + x*int(b[2]/2.0)
                    widthX = int(b[2]/2.0)
                    if x == 1:
                        widthX = b[2] - int(b[2]/2.0)
                    for y in range(2):
                        startY = b[1] + y*(int(b[3]/2.0))
                        widthY = int(b[3]/2.0)
                        if y == 1:
                            widthY = b[3] - int(b[3]/2.0)
                        
                        new_pilotes = []
                        for pil in b[4]:
                            if 0 <= pil.depart[0] - startX < widthX and 0 <= pil.depart[1] - startY < widthY:
                                new_pilotes.append(pil)
                        
                        first_dirs = [[Direction.SOUTH, Direction.WEST], [Direction.SOUTH, Direction.EAST], [Direction.NORTH, Direction.WEST], [Direction.NORTH, Direction.EAST]]
                        if i==0:
                            boxs[1].append([startX, startY, widthX, widthY, new_pilotes, first_dirs[x+2*y][0], widthY, x, y])
                            boxs[2].append([startX, startY, widthX, widthY, new_pilotes, first_dirs[x+2*y][1], widthX, x, y])
                        else:
                            dir1, dir2 = None, None
                            duration1, duration2 = 0, 0
                            
                            if b[7] != x and b[8] != y:
                                dir1 = Direction.WAIT
                                dir2 = Direction.WAIT
                            elif b[7] == x and b[8] == y:
                                dir1, dir2 = first_dirs[x+2*y][0], first_dirs[x+2*y][1]
                                duration1 = widthY
                                duration2 = widthX
                            else:
                                dir2 = Direction.WAIT
                                if b[7] == 0 and b[8] == 0:
                                    if x == 0 and y == 1:
                                        dir1 = Direction.WEST
                                        duration1 = widthX
                                    else:
                                        dir1 = Direction.SOUTH
                                        duration1 = widthY
                                elif b[7] == 1 and b[8] == 0:
                                    if x == 0 and y == 0:
                                        dir1 = Direction.SOUTH
                                        duration1 = widthY
                                    else:
                                        dir1 = Direction.EAST
                                        duration1 = widthX
                                elif b[7] == 0 and b[8] == 1:
                                    if x == 0 and y == 0:
                                        dir1 = Direction.WEST
                                        duration1 = widthX
                                    else:
                                        dir1 = Direction.NORTH
                                        duration1 = widthY
                                elif b[7] == 1 and b[8] == 1:
                                    if x == 0 and y == 1:
                                        dir1 = Direction.NORTH
                                        duration1 = widthY
                                    else:
                                        dir1 = Direction.EAST
                                        duration1 = widthX
                                        
                            boxs[i+1].append([startX, startY, widthX, widthY, new_pilotes, dir1, duration1, b[7], b[8]])
                            boxs[i+2].append([startX, startY, widthX, widthY, new_pilotes, dir2, duration2, b[7], b[8]])

        
        i+=1
                
    return boxs[1:-1]




def ecarter(pilotes, list_boxs, etape=1):
    boxes = list_boxs[etape-1]
    
    liste_steps = []
    for b in boxes:
        for i in range(b[6]):
            if i >= len(liste_steps):
                liste_steps.append(SolutionStep())
                
            for pil in b[4]:
                liste_steps[i][pil.index]  = b[5]
        
        inc = (0,0)
        if b[5] == Direction.NORTH:
            inc = (0,1)
        elif b[5] == Direction.SOUTH:
            inc = (0,-1)
        elif b[5] == Direction.WEST:
            inc = (-1,0)
        elif b[5] == Direction.EAST:
            inc = (1,0)
            
        for pil in b[4]:
             for _ in range(b[6]):
                 pil.move(inc)
            
                
    return liste_steps
            
    
                
            


def trouverSolution(file, optimizeMakespan = True, maxMakespan = 200, maxDistance = 10000, timeMax = 10,
                    repulsionMoy=0.3, repulsionVariation=0.1,
                    aleatoireMoy=0.2, aleatoireVariation=0.1,
                    deterministeMoy=30, deterministeVariation=12,
                    poissonMoy=0.2, poissonVariation=0.1):
    """
    Recherche une solution optimale (en makespan ou distance)
    Parameters
    ----------
    optimizeMakespan : TYPE, optional
        DESCRIPTION. The default is True. True -> optimiser le makespan / False -> optimiser la distance
    maxMakespan : TYPE, optional
        DESCRIPTION. The default is 200. Valeur maximale pour le makespan (si on optimise par rapport au makespan)
        Si on dépasse cette valeur, on recommence
    maxDistance : TYPE, optional
        DESCRIPTION. The default is 10000. Valeur maximale pour la distance (si on optimise par rapport à la distance)
        Si on dépasse cette valeur, on recommence
    timeMax : TYPE, optional
        DESCRIPTION. The default is 10. (en secondes)
        Temps utilisé pour chercher les solutions
    Returns
    -------
    None.
    """
    
    s = input("Nom de l'enregistrement du fichier ? (\"No\" pour ne pas sauvegarder)\n")

    # Chargement du fichier
    idb = InstanceDatabase("datasets.zip")

    i= idb[file]
    
    nx = 1 + max([i.start_of(r)[0] for r in range(i.number_of_robots)])
    ny = 1 + max([i.start_of(r)[1] for r in range(i.number_of_robots)])
    
    # On rajoute la marge
    margeX = nx
    margeY = ny
    tailleX = nx + 2*margeX
    tailleY = ny + 2*margeY
    print(nx, ny)
    
    
    dim = 1
    groups = [[], []]
    
    """
    for robot in pilotes:
        for ix in range(2):
            for iy in range(2):
                if int(ix*tailleBox/2.0) <= robot.depart[0] < int((ix*tailleBox/2.0)
    
    """
    # Recherche de la solution
    solution = None 
    
    pilotes = []
    obstacles_decales = [ (a+margeX,b+margeY) for (a,b) in i.obstacles]
        
    for r in range(i.number_of_robots):
        cible_decalee = (i.target_of(r)[0] + margeX, i.target_of(r)[1] + margeY)
        depart_decale = (i.start_of(r)[0] + margeX, i.start_of(r)[1] + margeY)
        pilotes.append(Pilote(r, tailleX, tailleY, cible_decalee, depart_decale, obstacles_decales))
    
    print("Pilotes initialisés")
    boxs = getBoxes(pilotes, margeX, margeY, nx, ny)
    print("Got boxes")
    #print(calculPriorites2(pilotes))
    makespanMini = maxMakespan
    distanceMini = maxDistance
    nbAmeliorations = 0
    nbEssais = 0
    
    tStart = time.time()
    while time.time() - tStart < timeMax:
        solution = Solution(i)
        
        # Nombre de robots arrivés à leur cible
        nbArrives = 0
        
        # MakeSpan et distance
        makespan = 0
        distance = 0
        
        # On met des params aléatoires
        [ elt.reset() for elt in pilotes]
        pr = repulsionMoy + repulsionVariation *(-1 +2*np.random.random())
        pa = aleatoireMoy + aleatoireVariation *(-1 +2*np.random.random())
        pd = deterministeMoy + deterministeVariation *(-1 +2*np.random.random())
        poisson = poissonMoy + poissonVariation *(-1 +2*np.random.random())
        [elt.params(pr,pa,pd, poisson) for elt in pilotes]
        
        # Variables utilisées à chaque step
        nbArrives = 0
        nbRobotsTotal = i.number_of_robots
        pilotesArrives = []
        pilotesActifs = [p for p in pilotes]
        cases_prises = [elt.p for elt in pilotes] # Cases inaccessibles
        
        needReset = False
        stepVideCount = 0
        stepVideMaxCount = 20
        
        print("début ecartement")
        for etape in range(len(boxs)):
            print(etape+1)
            liste_steps = ecarter(pilotes, boxs, etape+1)
            for step in liste_steps:
                makespan += 1
                solution.add_step(step)
                
        print("\n\nEcartement terminé")
        priorites = calculPriorites2(pilotes)[::-1]
        print(priorites)
        prio = 0
        """
        with SolutionZipWriter("outNOUVEAUTrump.zip") as szw:
                szw.add_solution(solution)
        print("Enregistrement terminé")
        """
        
        while not(needReset) and (nbArrives < nbRobotsTotal) and ((optimizeMakespan and makespan<makespanMini) or (not(optimizeMakespan) and distance<distanceMini)):
            step = SolutionStep()
            print(makespan)
            if len(priorites[prio]) == 0:
                prio += 1
                
            stepIsEmpty = True # Booléen qui nous permettra de ne pas ajouter des steps inutiles
              
            nbArrives = len(pilotesArrives)
            
            # Anciennes positions des robots, à enlever au prochain pas
            caseToRemove = []
            cases_prises = [(elt.p[0], elt.p[1], Direction.WAIT) for elt in pilotes] # Cases inaccessibles
            
            #random.shuffle(pilotesActifs)
            for index in priorites[prio]:
                monPilote = pilotes[index]
                pp = None # Action du robot (WAIT, SOUTH, NORTH, WEST, EAST)
                if monPilote.p == monPilote.cible: 
                    nbArrives += 1
                    #print("Le robot "+str(r)+" est arrivé à destination")
                    pp = Direction.WAIT
                    pilotesArrives.append(monPilote)
                    pilotesActifs.remove(monPilote)
                    print("nbArrives =", nbArrives)
                    priorites[prio].remove(index)
                    for p in pilotes:
                        p.ajouterObstacles([monPilote.p], recalculer = random.random() <= 0.1)
                else :
                    previousPosPilote = monPilote.p
                    pp = monPilote.pas_probabiliste(cases_prises)
                    if pp != Direction.WAIT: # Si le robot bouge, le pas n'est pas inutile
                        stepIsEmpty = False
                        distance += 1
                        #caseToRemove.append(previousPosPilote)
                        cases_prises.append((monPilote.p[0], monPilote.p[1], Direction.WAIT))
                        
                        cases_prises.remove((previousPosPilote[0], previousPosPilote[1], Direction.WAIT))
                        cases_prises.append((previousPosPilote[0], previousPosPilote[1], monPilote.lastDir))
                step[monPilote.index] = pp
                
                
            if not(stepIsEmpty):
                makespan += 1
                stepVideCount = 0
                solution.add_step(step)
                
                # On enlève les anciennes positions des robots
                for pos in caseToRemove:
                    cases_prises.remove(pos)
            else:
                stepVideCount += 1
                if stepVideCount >= stepVideMaxCount:
                    needReset = True


        # On est sortis de la boucle
        nbEssais += 1
        print("arrives = ", nbArrives)
        # SI ON ARRIVE ICI C'EST QU'ON A TROUVE UNE SOLUTION
        if not(needReset) and ((optimizeMakespan and makespan<makespanMini) or (not(optimizeMakespan) and distance<distanceMini)):
            nbAmeliorations += 1
            print("")
            print("Trouvé une solution au "+str(nbEssais)+"ième essai | makespan = "+str(solution.makespan) + "   | distance = " + str(solution.total_moves))

            if optimizeMakespan:
                makespanMini = solution.makespan
            else:
                distanceMini = solution.total_moves
            
            """
            with SolutionZipWriter("outJoss"+str(solution.makespan)+".zip") as szw:
                szw.add_solution(solution)
            """
            
            if s.lower() != "no":
                # On crée le dossier s'il nexiste pas
                if not os.path.exists('solutions/' + file):
                    os.makedirs('solutions/' + file)
                if not os.path.exists('solutions/' + file + "/zip"):
                    os.makedirs('solutions/' + file + "/zip")
                    
                    
                filename = str(datetime.now()).split(".")[0].replace(" ", "_").replace("-", "_").replace(":", "_") + "_" + str(nbAmeliorations) + "___" + s 
                save_name = 'solutions/' + file + '/zip/' + 'out_' + filename + '.zip'#'solutions/' + file + "/zip/" + filename + ".zip"
                with SolutionZipWriter(save_name) as szw:
                    szw.add_solution(solution)
                    
                
                with zipfile.ZipFile(save_name, 'r') as zip_ref:
                    zip_ref.extractall("")
        
                list_of_files = glob.glob('solutions/*.json') # * means all if need specific format then *.csv
                latest_file = max(list_of_files, key=os.path.getctime).replace("\\", "/")
                shutil.move(latest_file, "solutions/" + file + "/" + filename + ".json")
                print("Saved to :", "solutions/" + file + "/" + filename + ".json")
            else:
                print("File not saved")
                
            try:
                validate(solution)
                print("solution validée par cgshop2021_pyutils.validate")
            except ZipReaderError as zre:
                print("Bad Zip:", zre)
            except InvalidSolutionError as ise:
                print("Bad Solution:", ise)
            except SolutionEncodingError as see:
                print("Bad Solution File:", see)
    
    print(nbArrives)
    
    
    
    
    
    
            
              
                

            




#"small_000_10x10_20_10.instance"
#"small_free_001_10x10_40_40.instance"
"universe_bgradiation_00009_100x100_80_8000"

trouverSolution("galaxy_cluster_00000_20x20_20_80.instance", maxMakespan = 400, optimizeMakespan = True,
                timeMax=60)
"""

b = getBoxes([], 10, 10)
i = 1
for line in b:
    print("\n")
    print("dim =", i)
    print(line)
    i+=1
    
"""
