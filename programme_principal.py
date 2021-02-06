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
from robotsCGSHOP2021.map import Map
from robotsCGSHOP2021.progressBar import ProgressBar
from datetime import datetime

import pickle
import ujson

def copyPickle(a):
    return pickle.loads(pickle.dumps(a, -1))

def copyJson(a):
    return ujson.loads(ujson.dumps(a))


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
    
    __slots__ = ('depart', 'p', 'index', 'cible', 'dernier_pas', 'lastDir', 'carte', 'potentiel', 'r', 'a', 'd', 'poisson', 'porteeRepulsion', 'attirancePotentiel')
    
    
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
        
        self.potentiel = None#np.zeros((nx,ny))
        
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
        
        
    def calculChemin(self, cible):
        self.carte.bfs(cible[0], cible[1], distmax=500000+dist(self.p, cible))
        
        
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
    
    
    def params(self,r,a,d, poisson, pr):
        self.r = r
        self.a = a
        self.d = d
        self.poisson = poisson
        self.porteeRepulsion = pr
        
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
                if self.porteeRepulsion >= dis > 0:
                    poids_direction[index(d)]-=REPULSION / dis
            
        """
        for c in self.aEviter :
            for d in decisions_possibles :
                dis = dist((c[0]-p[0], c[1]-p[1]),d)
                if self.porteeRepulsion >= dis > 0:
                    poids_direction[index(d)]-= 10*REPULSION / dis
        """
        
        for d in decisions_possibles :
            d_abs = (p[0]+d[0],p[1]+d[1])
            delta = self.carte.getValue(p) - self.carte.getValue(d_abs)
            
            # Aller dans la bonne direction
            if delta > 0: delta *= DETERMINISTE
            
            # Potentiel
            if not(self.potentiel is None):
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
        





def calculPriorites2(pilotes, pilotesActifs, pilotesArrives, optimizeMakespan=True, tailleGroupeMax = None, debug=False):
    if tailleGroupeMax is None: tailleGroupeMax = len(pilotes)
    
    pasEncoreArrives = [p.index for p in pilotesActifs]
    obstaclesPerm = [p.cible for p in pilotesArrives]
    dejaArrives = []
    groupes = []
    distance = 0
    
    prog = None
    arrives = 0
    groupe = 0
    if debug:
        prog = ProgressBar(len(pilotes), prefix = 'Groupe: ref', showEstimatedTime=False)
        prog.printTitle("Calcul Priorités 2")
    
    distancesOpti = []
    for p in pilotes:
        p.carte.resetMap()
        p.carte.setCible(p.cible)
        p.ajouterObstacles(obstaclesPerm, recalculer=True)
        #p.calculChemin(p.cible)
        distancesOpti.append(p.carte.getValue(p.p))
        
        if debug:
            arrives +=1
            if arrives % 10 == 0: prog.printP(arrives)
            
    if debug: prog.stop()
    
    while len(pasEncoreArrives) > 0:
        if debug:
            groupe += 1
            arrives = 0
            prog = ProgressBar(len(pasEncoreArrives), prefix = 'Groupe: ' + str(groupe) + ' ' * (3-len(str(groupe))), suffix = '   Robots restants = ' + str(len(pasEncoreArrives)), showEstimatedTime=False)
        
        obstacles = [pilotes[index].cible for index in pasEncoreArrives] + obstaclesPerm + [pilotes[index].p for index in dejaArrives] 
        for index in pasEncoreArrives:
            pilotes[index].carte.resetMap()
            pilotes[index].ajouterObstacles(obstacles, recalculer=True)
            #pilotes[index].calculChemin(pilotes[index].cible)
            
            if debug:
                arrives +=1
                if arrives % 10 == 0: prog.printP(arrives)
                
        if debug: prog.stop()
        
        possibles = []
        
        
        
        for index in pasEncoreArrives:
            if pilotes[index].carte.getValue(pilotes[index].p) >= 0: # s'il existe un chemin
                possibles.append(index)
                
                
        if len(possibles) > 0:
            possiblesSorted = sorted(possibles, key=lambda index: pilotes[index].carte.getValue(pilotes[index].p) - distancesOpti[index])
            possiblesSorted = possiblesSorted[:tailleGroupeMax]
            if optimizeMakespan:
                groupes.append([])
                
            for index in possiblesSorted:
                distance += pilotes[index].carte.getValue(pilotes[index].p)
                pasEncoreArrives.remove(index)
                dejaArrives.append(index)
                if optimizeMakespan:
                    groupes[-1].append(index)
                else:
                    groupes.append([index])           
        else:
            return None
              
    return groupes



def calculPrioritesWithoutStart(pilotes, nx, ny, margeX, margeY, debug = False):
    pasEncoreArrives = [p.index for p in pilotes]
    dejaArrives = []
    obstacles = []
    groupes = []
    distance = 0
    
    prog = None
    groupe = 0
    arrives = 0
    if debug:
        prog = ProgressBar(len(pilotes), prefix = 'Cibles ext.', showEstimatedTime=False)
        prog.printTitle("Calcul Priorités pour l'écartement")
    
    for p in pilotes:
        p.p = getCibleExterieure(p.cible, nx, ny, margeX, margeY)
        if debug:
            arrives +=1
            if arrives % 10 == 0: prog.printP(arrives)           
    if debug: prog.stop()
    
    while len(pasEncoreArrives) > 0:
        groupes.append([])
        
        if debug:
            groupe += 1
            arrives = 0
            prog = ProgressBar(len(pasEncoreArrives), prefix = 'Groupe: ' + str(groupe) + ' ' * (3-len(str(groupe))), suffix = '   Robots restants = ' + str(len(pasEncoreArrives)), showEstimatedTime=False)
        
        obstacles = [pilotes[index].cible for index in pasEncoreArrives] #+ [pilotes[index].p for index in dejaArrives]
        for index in pasEncoreArrives:
            pilotes[index].carte.resetMap()
            pilotes[index].ajouterObstacles(obstacles, recalculer=True)
            if debug:
                arrives +=1
                if arrives % 10 == 0: prog.printP(arrives)
        ajout = False
        
        for index in copy(pasEncoreArrives):
            if pilotes[index].carte.getValue(pilotes[index].p) >= 0: # s'il existe un chemin
                distance += pilotes[index].carte.getValue(pilotes[index].p)
                groupes[-1].append(index)
                pasEncoreArrives.remove(index)
                dejaArrives.append(index)
                ajout = True
                
        if debug: prog.stop()
                
                
                
        if not(ajout):
            print("IMPOSSIBLE pour les robots:", pasEncoreArrives)
            return None
                
    
    return groupes





def ecarterProba(pilotes, nx, ny, margeX, margeY, temps, priorites=None, debug=False, attirancePotentiel=25, shuffleMin=0):
    if debug:
        print('Ecartement des robots en ' + str(temps) + ' étapes...')
    
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
        p.potentiel = potentiel
        p.r = 5
        p.a = 1
        p.d = 15
        p.poisson = 3
        p.porteeRepulsion = 4
        p.attirancePotentiel = 25
        
    
    
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
    stop = False
    ecartementFini = None
        
    while not(needReset) and (not(stop) or makespan < shuffleMin) and makespan < temps:
        step = {}
        #step = SolutionStep()
        #print(makespan)
    
        
        if not(priorites is None) and newPrio:
            newPrio = False
            
            prog = None
            counter = 0
            if debug:
                prog = ProgressBar(len(pilotes), prefix = 'Groupe: ' + str(prio+1))
            
            aEviterPrio = [pilotes[index].cible for index in priorites[prio]]
            aEviterChemins = []
            for index in priorites[prio]:
                p = pilotes[index]
                p.carte.setAEviter([x for x in aEviterPrio if x != p.cible])
                p.calculChemin(p.cible)
                aEviterChemins += p.carte.getShortestPath(p.p)
                p.potentiel = None
                
                p.attirancePotentiel = 0
                p.r = 0
                p.a = 0
                p.d = 50
                p.poisson = 0
                p.porteeRepulsion = 1
                
                if debug:
                    counter += 1
                    if counter % 10 == 0: prog.printP(counter)
                
            for p in pilotes:
                if not(p.index in priorites[prio]):
                    p.carte.resetMap()
                    cible_ext = getCibleExterieure(p.cible, nx, ny, margeX, margeY)
                    p.carte.setAEviter(aEviterChemins)
                    p.calculChemin(cible_ext)
                    
                    if debug:
                        counter += 1
                        if counter % 10 == 0: prog.printP(counter)
                        
            if debug: prog.stop()
                
            
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
                if monPilote.index in priorites[prio]:
                    priorites[prio].remove(monPilote.index)
                for p in pilotes:
                    p.ajouterObstacles([monPilote.p], recalculer = p.index in priorites[prio])
                    
                if len(priorites[prio]) == 0:
                    newPrio = True
                    prio += 1
                    
                    if prio == len(priorites) - 1:
                       newPrio = False
                       stop = True
                       ecartementFini = makespan
                    
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
        del p.potentiel
        p.potentiel = None
        #p.potentiel = potentielNul
      
    if debug: print("")
    if makespan < temps and debug:
        print("Ecartement fini plus tôt, en " + str(ecartementFini) + " étapes. A duré au total " + str(makespan) + " étapes.")
        
    return liste_steps, listePositionsRobots





def nettoyage_steps(steps, pilotes, suivi, debug=False):
    if debug:
        print("Nettoyage de la solution")
    
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
    numero = 1
    while first or d > 0:
        if debug: print("Passage " + str(numero) + " ... ", end='')
        numero += 1
        
        d = 0
        first = False
        for x in range(nx):
            for y in range(ny):
                dernier_visiteur = T[0,x,y]
                derniere_visite = 0
                vide =  (dernier_visiteur ==-1)
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
                        
                        
        if debug: print(d)
                    
    #j=0
    del T
    
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
    
              
    
   
def getSolution(i, liste_steps):
    s = Solution(i)
    
    for step_dict in liste_steps:
        step = SolutionStep()
        for robot in step_dict.keys():
            step[robot] = step_dict[robot]
        s.add_step(step)
        
    return s


def saveSolution(solution, file, s, nbAmeliorations):
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
        print('\033[32m' + "Saved to :", "solutions/" + file + "/" + filename + ".json" + '\033[0m')
    else:
        print('\033[32m' + "File not saved" + '\033[0m')
            


def trouverSolution(file, optimizeMakespan = True, maxMakespan = 200, maxDistance = 10000, timeMax = 10,
                    repulsionMin=1, repulsionMax=3,
                    aleatoireMin=1, aleatoireMax=2,
                    deterministeMin=30, deterministeMax=40,
                    poissonMin=2, poissonMax=6,
                    porteeRepulsionMin=2, porteeRepulsionMax=4,
                    maxSimultanementMin=None, maxSimultanementMax=None,
                    anticipationMin = 0, anticipationMax=0,
                    useShuffle=True, usePriorite=True, useRollback=True,
                    shuffleMin=None, shuffleMax=None,
                    rollbackMaxCount = None, waitBeforeRollback = None,
                    probaRecalcul=None, penalisation=3,
                    margeX=None, margeY=None,
                    debug=False, tailleGroupePrioritesMax=None, stepVideMaxCount = 20,
                    attiranceEcartement = 30):
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
    
    # On récupère le nom du fichier pour l'enregistrement des solutions
    s = input("Nom de l'enregistrement du fichier ? (\"No\" pour ne pas sauvegarder)\n")
    
    if not(optimizeMakespan):
        useRollback = False
        probaRecalcul = 1.0
        stepVideMaxCount = 1000
    
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
    nbRobotsTotal = i.number_of_robots
    pilotes = [] 
    for r in range(i.number_of_robots):
        cible_decalee = (i.target_of(r)[0] + margeX, i.target_of(r)[1] + margeY)
        depart_decale = (i.start_of(r)[0] + margeX, i.start_of(r)[1] + margeY)
        pilotes.append(Pilote(r, tailleX, tailleY, cible_decalee, depart_decale, obstacles_decales))
        pilotes[-1].carte.penalisation = penalisation
        
    
    
    # On définit les bornes pour le nombre d'étapes de l'écartement (on écarte pendant un temps entre shuffleMin et shuffleMax)
    if shuffleMax is None: shuffleMax = max(nx, ny)
    if shuffleMin is None: shuffleMin = min(max(int(nx/2.0), int(ny/2.0)), shuffleMax)    
    if not(useShuffle): shuffleMin, shuffleMax = 0, 0
    
    # On définit les paramètres pour le rollback
    if waitBeforeRollback is None: waitBeforeRollback = nx + ny
    if rollbackMaxCount is None: rollbackMaxCount = 3
    if not(useRollback) : waitBeforeRollback = 1000000
    
    # Quelques paramètres en plus
    if probaRecalcul is None: probaRecalcul = min(1.0, max(0.01, 40.0/len(pilotes)))
    if maxSimultanementMax is None: maxSimultanementMax = nbRobotsTotal
    if maxSimultanementMin is None: maxSimultanementMin = min(maxSimultanementMax, nbRobotsTotal)
    if tailleGroupePrioritesMax is None:
        if optimizeMakespan:
            tailleGroupePrioritesMax = nbRobotsTotal
        else:
            tailleGroupePrioritesMax = max(1, int(nbRobotsTotal**2 / 8000.0))
    newPrio = (calculPrioritesWithoutStart(pilotes, nx, ny, margeX, margeY, debug=debug))[::-1]
    if debug: print("")
        



    

    makespanMini = maxMakespan
    makespanRealMini = maxMakespan
    distanceMini = maxDistance
    distanceRealMini = maxDistance
    nbAmeliorations = 0
    nbEssais = 0
    
    tStart = time.time()
    while time.time() - tStart < timeMax:
        
        # Makespan, distance et nombre de robots arrivés à leur cible
        nbArrives = 0
        makespan = 0
        distance = 0
        
        
        # On réinitialise les pilotes
        for elt in pilotes:
            elt.reset()
        
        
        # On écarte les robots
        # On nettoie immédiatement car il y a beaucoup de déplacements à nettoyer et cela permet de réduire
        # l'incertitude sur la distance réelle
        shuffle = random.randint(shuffleMin, shuffleMax)
        liste_steps, listePositionsRobots = ecarterProba(pilotes, nx, ny, margeX, margeY, shuffle, priorites=copyJson(newPrio), debug=debug, attirancePotentiel=attiranceEcartement, shuffleMin=shuffleMin)
        liste_steps = nettoyage_steps(liste_steps, pilotes, listePositionsRobots)
        for step in liste_steps:
            makespan += 1
            distance += len(step)
        if debug: print("")
            
        
        # Pour mettre a jour a chaque step les obstacles
        nbArrives = 0
        cases_prises = [elt.p for elt in pilotes] # Cases inaccessibles
        pilotesArrives = []
        pilotesActifs = []
        for p in pilotes:
            if p.p == p.cible:
                pilotesArrives.append(p)
                nbArrives +=1
            else:
                pilotesActifs.append(p)
                
            
        # On calcule les priorités
        needReset = False
        priorites = [[elt.index for elt in pilotesActifs]]
        if usePriorite:
            priorites = calculPriorites2(pilotes, pilotesActifs, pilotesArrives, optimizeMakespan=optimizeMakespan, tailleGroupeMax=tailleGroupePrioritesMax, debug=debug)       
            # On regarde s'il existe un ordre de priorité qui fonctionne
            if priorites is None:
                needReset = True
            else:
                priorites = priorites[::-1]
           
        if needReset:
            print("IMPOSSIBLE: needReset")
        else:
            
            if True:
                obstacles = [elt.cible for elt in pilotesArrives]
                if not(optimizeMakespan): obstacles += [elt.p for elt in pilotesActifs]
                aEviter = []
                for i_prio in range(len(priorites)):
                    obstacles += aEviter
                    aEviter = [pilotes[index].cible for index in priorites[i_prio]]
                    
                    # On recalcule le BFS pour tous les robots
                    for index in priorites[i_prio]:
                        p = pilotes[index]
                        p.carte.resetMap()
                        p.carte.setAEviter([x for x in aEviter if x != p.cible])
                        p.carte.setCible(p.cible)
                        p.carte.aEteCaclcule = False
                        
                        if optimizeMakespan:
                            p.ajouterObstacles(obstacles, recalculer=True)
                        else:
                            p.ajouterObstacles(obstacles, recalculer=False)
                            p.calculChemin(p.cible)
                            obstacles.remove(p.p)
              

            if True:
                prio = 0
                
                # Variables pour abandonner si on est dans un cul de sac
                needReset = False
                stepVideCount = 0
            
            
                # On règle les paramètres aléatoires
                pr = repulsionMin + (repulsionMax - repulsionMin) * np.random.random()
                pa = aleatoireMin + (aleatoireMax - aleatoireMin) * np.random.random()
                pd = deterministeMin + (deterministeMax - deterministeMin) * np.random.random()
                poisson = poissonMin + (poissonMax - poissonMin) * np.random.random()
                ppr = round(porteeRepulsionMin + (porteeRepulsionMax - porteeRepulsionMin) *np.random.random())
                maxSimultanement = round(maxSimultanementMin + (maxSimultanementMax - maxSimultanementMin) * np.random.random())
                anticipation = round(anticipationMin + (anticipationMax - anticipationMin) * np.random.random())
                
                
                if not(optimizeMakespan):
                    anticipation = 0
                    pr = 0
                    pa = 0.05
                    pd = 100
                    poisson = 0
                    ppr = 1
                
                
                for elt in pilotes:
                    elt.params(pr,pa,pd, poisson, ppr)
                
                
                
                
                    
                # ===== SAVE en cas de rollback ===== #
                rollbackCount = 0
                lastArrival = makespan
                savePrio = prio
                saveDistance = distance
                savePriorites = deepcopy(priorites)
                saveIndexActifs = [elt.index for elt in pilotesActifs]
                saveIndexArrives = [elt.index for elt in pilotesArrives]
                nbRollbacks = 0
                
                #print(pilotes[priorites[0][0]].carte.show())
                
                prog = None
                if debug:
                    prog = ProgressBar(nbRobotsTotal)
                    prog.printTitle("Nombre de robots arrivés")
                    prog.printP(nbArrives)
                
                
                while not(needReset) and (nbArrives < nbRobotsTotal) and ((optimizeMakespan and makespan<makespanMini) or (not(optimizeMakespan) and distance<distanceMini)):
                    step = {}
                    #print(makespan, nbArrives)
                    
                    if debug and makespan % 10 == 0:
                        prog.printP(nbArrives)
                        
                    
                    if len(priorites[prio]) <= anticipation and len(priorites) > prio+1:
                        for elt in priorites[prio]:
                             priorites[prio+1].append(elt)
                             savePriorites[prio+1].append(elt)
                        prio += 1
                        savePrio += 1
                    
                        
                    stepIsEmpty = True # Booléen qui nous permettra de ne pas ajouter des steps inutiles
                      
                    nbArrives = len(pilotesArrives)
                    
                    # Anciennes positions des robots, à enlever au prochain pas
                    cases_prises = [(elt.p[0], elt.p[1], Direction.WAIT) for elt in pilotes] # Cases inaccessibles
                    
                    #random.shuffle(pilotesActifs)
                    for index in priorites[prio][:maxSimultanement]:
                    #for monPilote in pilotesActifs:#priorites[prio]:
                        #index = monPilote.index
                        monPilote = pilotes[index]
                        pp = None # Action du robot (WAIT, SOUTH, NORTH, WEST, EAST)
                        if monPilote.p == monPilote.cible:
                            
                            # SAUVERGARDE Pour le Rollback
                            lastArrival = makespan
                            rollbackCount = 0
                            saveDistance = distance
                            savePrio = prio
                            saveIndexArrives.append(index)
                            saveIndexActifs.remove(index)
                            if index in priorites[prio]: savePriorites[prio].remove(index)
                            
                            # Le robot arrive et attends
                            nbArrives += 1
                            pp = Direction.WAIT
                            if monPilote in pilotesActifs:
                                pilotesArrives.append(monPilote)
                                pilotesActifs.remove(monPilote)
                            if index in priorites[prio]: priorites[prio].remove(index)
                            
                            # On mets à jour les obstacles des autres
                            if optimizeMakespan:
                                for p in pilotesActifs:
                                    p.ajouterObstacles([monPilote.p], recalculer = random.random() <= probaRecalcul*nbRobotsTotal/float(len(pilotesActifs)))
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
                        rollbackCount += 1
                        nbRollbacks += 1
                        #liste_steps = nettoyage_steps(liste_steps, pilotes, listePositionsRobots)
                        #solution = getSolution(i, liste_steps)
                        #saveSolution(solution, file, "rollback" + str(rollbackCount) + "_" + str(makespan), nbAmeliorations)
                        if rollbackCount >= rollbackMaxCount:
                            #print("\n", rollbackMaxCount, "ROLLBACK d'affilé -> reset\n")
                            needReset = True
                        else:
                            makespan = lastArrival
                            distance = saveDistance
                            
                            # On enlève les steps qu'on rollback ainsi que l historique des positions
                            liste_steps = liste_steps[:lastArrival]
                            listePositionsRobots = listePositionsRobots[:lastArrival+1]
                            
                            # On replace les robots
                            for j in range(len(listePositionsRobots[-1])):
                                pilotes[j].p = listePositionsRobots[-1][j]
                            
                            # On remet en place les priorités et les robots en déplacement
                            prio = savePrio
                            priorites = deepcopy(savePriorites)
                            pilotesActifs = [pilotes[index] for index in saveIndexActifs]
                            pilotesArrives = [pilotes[index] for index in saveIndexArrives]
                            
                            # On recalcule tous les BFS
                            obstacles = [p.cible for p in pilotes]
                            
                            if rollbackCount > int(rollbackCount/2.0):
                                obstacles += [p.p for p in pilotesActifs]
                                
                            for p in pilotesActifs:
                                p.carte.resetMap()
                                p.carte.setCible(p.cible)
                                p.ajouterObstacles(obstacles, recalculer=True)
                                
        
        
                
                
        
                # On est sortis de la boucle
                nbEssais += 1
                newSol = False
                
                if debug:
                    prog.printP(nbArrives)
                    prog.stop(finish=False)
                    
                #print("arrives = ", nbArrives)
                # SI ON ARRIVE ICI C'EST QU'ON A TROUVE UNE SOLUTION
                if not(needReset) and ((optimizeMakespan and makespan<makespanMini) or (not(optimizeMakespan) and distance<distanceMini)):
                    beforeMakespan = makespan
                    beforeDistance = distance
                    
                    liste_steps = nettoyage_steps(liste_steps, pilotes, listePositionsRobots)
                    solution = getSolution(i, liste_steps)
                    
                    makespan = solution.makespan
                    distance = solution.total_moves
                    
                    if ((optimizeMakespan and makespan<makespanRealMini) or (not(optimizeMakespan) and distance<distanceRealMini)):
                        nbAmeliorations += 1
                        newSol = True
                        print("")
                        print('\033[32m' + "Trouvé une solution au "+str(nbEssais)+"ième essai | makespan = "+str(solution.makespan) + "   | distance = " + str(solution.total_moves) + '\033[0m')
                        print("repu=" + str(pr) + ", alea=" + str(pa) + ", deter=" + str(pd) + ", poisson=" + str(poisson) + ", portee repu=" + str(ppr))
                        print("shuffle=" + str(shuffle) + ", anticipation=" + str(anticipation) + ", nbRollBacks=" + str(nbRollbacks) + ", maxSimultanement=" + str(maxSimultanement))
                        
                        if optimizeMakespan:
                            makespanMini = beforeMakespan
                            makespanRealMini = makespan
                        else:
                            distanceMini = beforeDistance
                            distanceRealMini = distance
                        
            
            
                        saveSolution(solution, file, s, nbAmeliorations)            
                            
                        try:
                            validate(solution)
                            print("solution validée par cgshop2021_pyutils.validate")
                        except ZipReaderError as zre:
                            print("Bad Zip:", zre)
                        except InvalidSolutionError as ise:
                            print("Bad Solution:", ise)
                        except SolutionEncodingError as see:
                            print("Bad Solution File:", see)
                            
                    del solution
            
            
                if not(newSol):
                    print("")
                    print('\033[31m' + str(nbEssais) +": " + str(nbArrives) + " robots arrivés avec makespan=" + str(makespan) + " et distance=" + str(distance) + '\033[0m')
                    print('\033[32m' + "Best solution: makespan=" + str(makespanRealMini) + " et distance=" + str(distanceRealMini) + '\033[0m')
                    print("repu=" + str(pr) + ", alea=" + str(pa) + ", deter=" + str(pd) + ", poisson=" + str(poisson) + ", portee repu=" + str(ppr))
                    print("shuffle=" + str(shuffle) + ", anticipation=" + str(anticipation) + ", nbRollBacks=" + str(nbRollbacks) + ", maxSimultanement=" + str(maxSimultanement))
                           
    
                del liste_steps
                del savePriorites
                del priorites
                del pilotesActifs
                del saveIndexActifs
                del pilotesArrives
                del saveIndexArrives
                del listePositionsRobots
                if debug: print("\n\n")
            
              
                

            




#"small_000_10x10_20_10.instance"
#"small_free_001_10x10_40_40.instance"
#"universe_bgradiation_00009_100x100_80_8000"
#galaxy_cluster_00000_20x20_20_80
#galaxy_cluster2_00003_50x50_25_625

"""
trouverSolution("large_free_002_100x100_20_2000.instance", optimizeMakespan = True, maxMakespan = 3500, maxDistance = 100000, timeMax = 11000,
                    repulsionMin=4, repulsionMax=15,
                    aleatoireMin=1, aleatoireMax=5,
                    deterministeMin=30, deterministeMax=75,
                    poissonMin=5, poissonMax=20,
                    porteeRepulsionMin=2, porteeRepulsionMax=6,
                    maxSimultanementMin=1000, maxSimultanementMax=2000,
                    anticipationMin = 0, anticipationMax=10,
                    useShuffle=True, usePriorite=True, useRollback=True,
                    shuffleMin=55, shuffleMax=85,
                    rollbackMaxCount = 2, waitBeforeRollback = 200,
                    probaRecalcul=1.0, penalisation=3,
                    margeX=40, margeY=40, attiranceEcartement = 50,
                    debug=True, tailleGroupePrioritesMax=2000, stepVideMaxCount = 200)
"""