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


def last_index(liste, elt):
    for i in range(len(liste)-1,-1,-1):
        if liste[i] == elt: return i
    raise ExceptionError

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
        
        self.potentiel = np.zeros((nx,ny))
        
        self.r = 0.3
        self.a = 0.2
        self.d = 4
        self.poisson = 0.2
        self.porteeRepulsion = 2
        self.attirancePotentiel = 1
        
    def reset(self):
        self.p =self.depart
        self.dernier_pas =ZERO
        self.carte.resetMap()
        #self.calculerChemin()
        
        
    def ajouterObstacles(self, liste_obs, recalculer=True):
        self.carte.ajouterObstacles(liste_obs)
        if recalculer:
            self.carte.bfs_nouveaux_obstacles()
            
            
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
            
            # Aller dans la bonne direction
            if delta > 0: delta *= DETERMINISTE
            
            # Potentiel
            delta += self.attirancePotentiel * (self.potentiel[d_abs] - self.potentiel[p])
            
            # Rester collé à un robot voisin
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



def calculPrioritesWithoutStart(pilotes, nx, ny, margeX, margeY):
    pasEncoreArrives = [p.index for p in pilotes]
    dejaArrives = []
    obstacles = []
    groupes = []
    distance = 0
    
    for p in pilotes:
        p.p = getCibleExterieure(p.cible, nx, ny, margeX, margeY)
    
    while len(pasEncoreArrives) > 0:
        groupes.append([])
        
        obstacles = [pilotes[index].cible for index in pasEncoreArrives] #+ [pilotes[index].p for index in dejaArrives]
        for index in pasEncoreArrives:
            pilotes[index].carte.resetMap()
            pilotes[index].ajouterObstacles(obstacles, recalculer=True)
        ajout = False
        
        for index in copy(pasEncoreArrives):
            if 100 >= pilotes[index].carte.getValue(pilotes[index].p) >= 0: # s'il existe un chemin
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



def ecarterProba(pilotes, nx, ny, margeX, margeY, temps, priorites=None):
    AJOUT_POTENTIEL = nx + ny
    
    potentielNul = np.zeros((nx+2*margeX, ny+2*margeY))   
    potentiel = np.ones((nx+2*margeX, ny+2*margeY)) * (int(nx/2.0) + int(ny/2.0) + AJOUT_POTENTIEL)
    milieu = (margeX + int(nx/2.0), margeY + int(ny/2.0))
    
    # Milieu
    for x in range(margeX, margeX + nx):
        for y in range(margeY, margeY + ny):
            potentiel[(x,y)] = dist((x,y), milieu)
            
    # Bords
    for x in range(0, 2*margeX + nx):
        potentiel[(x,0)] = 0
        potentiel[(x,2*margeY+ny-1)] = 0
    for y in range(0, 2*margeY + ny):
        potentiel[(0,y)] = 0
        potentiel[(2*margeX+nx-1,y)] = 0
        
        
    for p in pilotes:
        p.carte.resetMap()
        cible_ext = getCibleExterieure(p.cible, nx, ny, margeX, margeY)
        p.carte.bfs(cible_ext[0], cible_ext[1])
        p.potentiel = potentiel
        p.r = 5
        p.a = 1
        p.d = 50
        p.poisson = 3
        p.porteeRepulsion = 4
        p.attirancePotentiel = 10
        
        
    makespan = 0
    distance = 0
    liste_steps = []
    stepVideMaxCount = 10
    needReset = False
    stepVideCount = 0
      
    listePositionsRobots = [[elt.p for elt in pilotes]]
    
    prio = 0
    newPrio = True
    pilotesArrives = []
    pilotesActifs = [elt for elt in pilotes]
        
    while not(needReset) and makespan < temps:
        step = {}
        #step = SolutionStep()
        #print(makespan)
        
        if not(priorites is None) and newPrio:
            newPrio = False
            for index in priorites[prio]:
                p = pilotes[index]
                p.carte.bfs(p.cible[0], p.cible[1])
                p.potentiel = potentielNul
            
            pilotesActifs = []
            for groupe_prio in priorites[prio:]:
                for index_robot in groupe_prio:
                    pilotesActifs.append(pilotes[index_robot])
                
        stepIsEmpty = True # Booléen qui nous permettra de ne pas ajouter des steps inutiles
            
        # Anciennes positions des robots, à enlever au prochain pas
        cases_prises = [(elt.p[0], elt.p[1], Direction.WAIT) for elt in pilotes] # Cases inaccessibles
            
        for monPilote in pilotesActifs:
            previousPosPilote = monPilote.p
            pp = monPilote.pas_probabiliste(cases_prises)
            
            if pp != Direction.WAIT: # Si le robot bouge, le pas n'est pas inutile
                stepIsEmpty = False
                distance += 1
                #caseToRemove.append(previousPosPilote)
                cases_prises.append((monPilote.p[0], monPilote.p[1], Direction.WAIT))
                        
                cases_prises.remove((previousPosPilote[0], previousPosPilote[1], Direction.WAIT))
                cases_prises.append((previousPosPilote[0], previousPosPilote[1], monPilote.lastDir))
                
            if not(priorites is None) and monPilote.index in priorites[prio] and monPilote.p == monPilote.cible:
                pilotesArrives.append(monPilote)
                pilotesActifs.remove(monPilote)
                if index in priorites[prio]:
                    priorites[prio].remove(index)
                for p in pilotes:
                    p.ajouterObstacles([monPilote.p], recalculer = p.index in priorites[prio])
                    
                if len(priorites[prio]) == 0:
                    newPrio = True
                    prio += 1
                    
            step[monPilote.index] = pp
                
                
        if not(stepIsEmpty):
            makespan += 1
            stepVideCount = 0
            liste_steps.append(step)
            listePositionsRobots.append([elt.p for elt in pilotes])
        else:
            stepVideCount += 1
            if stepVideCount >= stepVideMaxCount:
                needReset = True
                
           
            
    for p in pilotes:
        p.potentiel = potentielNul
      
    return liste_steps, listePositionsRobots





def nettoyage_steps(steps, pilotes, suivi):
    nx = pilotes[0].carte.nx
    ny = pilotes[0].carte.ny
    nb_robots = len(pilotes)
    makespan = len(steps)

    T = np.zeros((makespan+1,nx,ny),dtype=int)-1


    for t in range(len(steps)+1) :
        for r in range(nb_robots):
            T[t,suivi[t][r][0],suivi[t][r][1]]=r

    
    first = True
    d = 0
    while first or d > 0:
        d = 0
        first = False
        for x in range(nx):
            for y in range(ny):
                dernier_visiteur = T[0,x,y]
                derniere_visite = 0
                vide =  (dernier_visiteur ==-1 )
                for t in range(1,makespan+1):
                    contenu = T[t,x,y]
                    
                    if contenu ==-1 :
                        vide = True
                    elif contenu == dernier_visiteur and vide :
                        vide = False
                        
                        for pas in range(derniere_visite,t):
                            if dernier_visiteur in steps[pas].keys():
                                d += 1
                                del steps[pas][dernier_visiteur]
                        for pas in range(derniere_visite+1,t):
                            T[pas,suivi[pas][dernier_visiteur][0],suivi[pas][dernier_visiteur][1]] = -1
                            suivi[pas][dernier_visiteur] = (x,y)
                        vide = False
                        derniere_visite = t
                    else :
                        vide = False
                        dernier_visiteur = contenu
                        derniere_visite = t
                        
                    
    #j=0
    for i in reversed(range(len(steps))):
        if not steps[i]:
            steps.pop(i)
            #j+=1
    #print("gagné "+ str(j) +" en makespan")

    return steps



def getCibleExterieure(cible, nx, ny, margeX, margeY):
    ix = (cible[0] - int(nx/2.0) - margeX )
    iy = (cible[1] - int(ny/2.0) - margeY)
    
    cibleExtX = int(margeX/2.0) + (ix>0)*(nx + margeX)
    cibleExtY = int(margeY/2.0) + (iy>0)*(ny + margeY)
    
    if abs(ix) <= abs(iy):
        return (cible[0], cibleExtY)
    else:
        return (cibleExtX, cible[1])
    
            

def nettoyage_stepsFonctionnePas(steps, pilotes, suivi):
    nx = pilotes[0].carte.nx
    ny = pilotes[0].carte.ny
    nb_robots = len(pilotes)
    makespan = len(steps)

    T = np.zeros((makespan+1,nx,ny),dtype=np.short)-1


    for t in range(len(steps)+1) :
        for r in range(nb_robots):
            T[t,suivi[t][r][0],suivi[t][r][1]]=r

    
    #print(T)
    cycles = {}
    for x in range(nx):
        for y in range(ny):
            dernier_visiteur = T[0,x,y]
            derniere_visite = 0
            
            visiteurs = []
            visites = []
            for t in range(0,makespan+1):
                contenu = T[t,x,y]
                
                if contenu ==-1 :
                    pass
                elif len(visiteurs) == 0:
                    visiteurs = [contenu]
                    visites = [t]
                elif contenu in visiteurs:
                    index_derniere_visite = last_index(visiteurs, contenu)
                    if not contenu in cycles: cycles[contenu] = []
                    cycles[contenu].append([(x,y), visites[index_derniere_visite], t, visiteurs[index_derniere_visite+1:], visites[index_derniere_visite+1:]])
                else:
                    visiteurs.append(contenu)
                    visites.append(t)
                    

    for robot in cycles.keys():
        i_c = 0
        while not(cycles[robot] is None) and i_c < len(cycles[robot]):
            #for i_c in range(len(cycles[robot])):
            cycle = cycles[robot][i_c]
            (x,y) = cycle[0]
            derniere_visite = cycle[1]
            t = cycle[2]
            visiteurs = cycle[3]
            visites = cycle[4]
            
            if len(visiteurs) == 0:
                for pas in range(derniere_visite,t):
                    if robot in steps[pas].keys():
                        del steps[pas][robot]
                for pas in range(derniere_visite+1,t):
                    T[pas,suivi[pas][robot][0],suivi[pas][robot][1]] = -1
                    suivi[pas][robot] = (x,y)
                    
                j = i_c+1
                while not(cycles[robot] is None) and j < len(cycles[robot]):
                    autre_cycle = cycles[robot][j]
                    if derniere_visite <= autre_cycle[1] <= t or derniere_visite <= autre_cycle[2] <= t:
                        cycles[robot].pop(j)
                    else:
                        j+=1
            
                cycles[robot].pop(i_c)
            else:
                i_c += 1
      
    keys = [elt for elt in cycles.keys()]
    for key in keys:
        if len(cycles[key]) == 0:
            cycles.pop(key,None)
    print(cycles)
    return steps

            
    
   
def getSolution(i, liste_steps):
    s = Solution(i)
    
    for step_dict in liste_steps:
        step = SolutionStep()
        for robot in step_dict.keys():
            step[robot] = step_dict[robot]
        s.add_step(step)
        
    return s
            


def trouverSolution(file, optimizeMakespan = True, maxMakespan = 200, maxDistance = 10000, timeMax = 10,
                    repulsionMoy=2, repulsionVariation=1,
                    aleatoireMoy=1.5, aleatoireVariation=1,
                    deterministeMoy=30, deterministeVariation=12,
                    poissonMoy=5, poissonVariation=3,
                    useShuffle=True, usePriorite=True, useRollback=True,
                    shuffleMin=None, shuffleMax=None,
                    rollbackMaxCount = None, waitBeforeRollback = None,
                    probaRecalcul=None, rayon=None,
                    margeX=None, margeY=None):
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
    
    # Chargement du fichier
    idb = InstanceDatabase("datasets.zip")
    i= idb[file]
    
    
    
    # On récupère les dimensions de la map
    nx = 1 + max([i.start_of(r)[0] for r in range(i.number_of_robots)])
    ny = 1 + max([i.start_of(r)[1] for r in range(i.number_of_robots)])
    
    # On agrandit la map avec les marges
    if margeX is None: margeX = int(2*nx/3.0)
    if margeY is None: margeY = int(2*ny/3.0)
    tailleX = nx + 2*margeX
    tailleY = ny + 2*margeY
    
    # On récupère les obstacles que l'on décale à cause de la marge
    obstacles_decales = [ (a+margeX,b+margeY) for (a,b) in i.obstacles]
    
    # On récupère tous les robots que l'on décale également
    pilotes = [] 
    for r in range(i.number_of_robots):
        cible_decalee = (i.target_of(r)[0] + margeX, i.target_of(r)[1] + margeY)
        depart_decale = (i.start_of(r)[0] + margeX, i.start_of(r)[1] + margeY)
        pilotes.append(Pilote(r, tailleX, tailleY, cible_decalee, depart_decale, obstacles_decales))
        
    
    
    # On définit les bornes pour le nombre d'étapes de l'écartement (on écarte pendant un temps entre shuffleMin et shuffleMax)
    if shuffleMax is None: shuffleMax = max(nx, ny)
    if shuffleMin is None: shuffleMin = min(max(int(nx/2.0), int(ny/2.0)), shuffleMax)    
    if not(useShuffle): shuffleMin, shuffleMax = 0, 0
    
    # On définit els paramètres pour le rollback
    if waitBeforeRollback is None: waitBeforeRollback = nx + ny
    if rollbackMaxCount is None: rollbackMaxCount = 3
    if not(useRollback) : waitBeforeRollback = 1000000
    
    # Quelques paramètres en plus
    if probaRecalcul is None: probaRecalcul = min(1.0, max(0.01, 40.0/len(pilotes)))
    if rayon is None: rayon = int(max(nx, ny) / 5.0)
    
    newPrio = (calculPrioritesWithoutStart(pilotes, nx, ny, margeX, margeY))[::-1]
        
        
    
    # On récupère le nom du fichier pour l'enregistrement des solutions
    s = input("Nom de l'enregistrement du fichier ? (\"No\" pour ne pas sauvegarder)\n")


    

    makespanMini = maxMakespan
    distanceMini = maxDistance
    nbAmeliorations = 0
    nbEssais = 0
    
    tStart = time.time()
    while time.time() - tStart < timeMax:
        #solution = Solution(i)
        
        # Nombre de robots arrivés à leur cible
        nbArrives = 0
        
        # MakeSpan et distance
        makespan = 0
        distance = 0
        
        # On met des params aléatoires
        [ elt.reset() for elt in pilotes]
        
        liste_steps, listePositionsRobots = ecarterProba(pilotes, nx, ny, margeX, margeY, random.randint(shuffleMin, shuffleMax), newPrio)
        for step in liste_steps:
            makespan += 1
            #solution.add_step(step)
            
        for p in pilotes:
            p.carte.bfs(p.cible[0], p.cible[1])
        
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
        
        
        
        priorites = [[elt.index for elt in pilotes]]
        prio = 0
        if usePriorite:
            priorites = calculPriorites2(pilotes)
            
            if priorites is None:
                needReset = True
            else:
                priorites = priorites[::-1]
        else:
            for p in pilotes:
                p.carte.bfs(p.cible[0], p.cible[1])
            
            
        rollbackCount = 0
        lastArrival = makespan
        
        while not(needReset) and (nbArrives < nbRobotsTotal) and ((optimizeMakespan and makespan<makespanMini) or (not(optimizeMakespan) and distance<distanceMini)):
            step = {}
            #step = SolutionStep()
            #print(makespan)
                
            
            if len(priorites[prio]) == 0:
                prio += 1
                """
                for p in pilotes:
                    if p in priorites[prio]:
                        p.attirancePotentiel = 0
                        p.potentiel = np.zeros((2*margeX+nx, 2*margeY+ny))
                    else:
                        p.attirancePotentiel = 500
                        p.potentiel = np.ones((2*margeX+nx, 2*margeY+ny)) * (rayon+100)
                        for x in range(-rayon, rayon +1):
                            for y in range(-rayon + abs(x), rayon - abs(x) + 1):
                                distance_from_cible = abs(x) + abs(y)
                                p.potentiel[(p.cible[0]+x, p.cible[1]+y)] = distance_from_cible
                                
                """
            
                
            stepIsEmpty = True # Booléen qui nous permettra de ne pas ajouter des steps inutiles
              
            nbArrives = len(pilotesArrives)
            
            # Anciennes positions des robots, à enlever au prochain pas
            cases_prises = [(elt.p[0], elt.p[1], Direction.WAIT) for elt in pilotes] # Cases inaccessibles
            
            #random.shuffle(pilotesActifs)
            for index in priorites[prio]:
            #for monPilote in pilotesActifs:#priorites[prio]:
                #index = monPilote.index
                monPilote = pilotes[index]
                pp = None # Action du robot (WAIT, SOUTH, NORTH, WEST, EAST)
                if monPilote.p == monPilote.cible:
                    lastArrival = makespan
                    rollbackCount = 0
                    nbArrives += 1
                    #print("Le robot "+str(r)+" est arrivé à destination")
                    pp = Direction.WAIT
                    pilotesArrives.append(monPilote)
                    pilotesActifs.remove(monPilote)
                    #print("nbArrives =", nbArrives)
                    if index in priorites[prio]:
                        priorites[prio].remove(index)
                    for p in pilotes:
                        p.ajouterObstacles([monPilote.p], recalculer = random.random() <= probaRecalcul)
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
                        
                if pp != Direction.WAIT:
                    step[monPilote.index] = pp
                
                
            if not(stepIsEmpty):
                makespan += 1
                stepVideCount = 0
                liste_steps.append(step)
                listePositionsRobots.append([elt.p for elt in pilotes])
                
                #print("liste", makespan, len(listePositionsRobots))
                #solution.add_step(step)
            else:
                stepVideCount += 1
                if stepVideCount >= stepVideMaxCount:
                    needReset = True
                    
                    
            if makespan - lastArrival > waitBeforeRollback:
                print("ROOOOOOOOLLLLLLLLLBACKKK !!!!!")
                rollbackCount += 1
                if rollbackCount >= rollbackMaxCount:
                    print("\n", rollbackMaxCount, "ROLLBACK d'affilé -> reset\n")
                    needReset = True
                else:
                    makespan = lastArrival
                    liste_steps = liste_steps[:lastArrival+1]
                    listePositionsRobots = listePositionsRobots[:lastArrival+2]
                    
                    for j in range(len(listePositionsRobots[-1])):
                        pilotes[j].p = listePositionsRobots[-1][j]


        
        liste_steps = nettoyage_steps(liste_steps, pilotes, listePositionsRobots)
        solution = getSolution(i, liste_steps)

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
#"universe_bgradiation_00009_100x100_80_8000"
#galaxy_cluster_00000_20x20_20_80
#galaxy_cluster2_00003_50x50_25_625

trouverSolution("small_free_001_10x10_40_40.instance", maxMakespan = 10000, optimizeMakespan = True,
                timeMax=60, usePriorite=True, useShuffle=True, shuffleMin=25, shuffleMax = 26, probaRecalcul=0, deterministeMoy=100)

