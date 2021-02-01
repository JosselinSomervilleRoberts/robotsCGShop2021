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
    
    def voisins(self,case):
        '''Renvoie les voisins sans obstacles de la case'''
        n = self.n
        (x,y)=case
        L = []
        if (x>0 and self.T[x-1,y] !=OBS) :
            L.append((x-1,y))
        if (x<n-1 and self.T[x+1,y] !=OBS) :
            L.append((x+1,y))
        if (y>0 and self.T[x,y-1] !=OBS) :
            L.append((x,y-1))
        if (y<n-1 and self.T[x,y+1] !=OBS) :
            L.append((x,y+1))
        return L
    
    def calculerChemin(self):
        L = [self.cible]
        self.T[self.cible]=0
        while len(L)>0 :
            c = L[0]
            if not(c in self.dicFils):
                self.dicFils[c] = []
            V = self.voisins(c)
            for v in V :
                if self.T[v]==-1 :
                    L.append(v)
                    if not(v in self.dicFils[c]):
                        self.dicFils[c].append(v)
                    self.T[v]=self.T[c]+1
            L.pop(0)
    
    def __init__(self, index, n, cible,depart,T):
        self.depart = depart
        self.p = depart #La position actuelle du robot
        self.index = index
        self.cible = cible #La ou le robot doit aller
        self.n = n
        self.T = T
        self.dernier_pas = ZERO
        self.lastDir = Direction.WAIT
        #Le tableau T est un tableau d'entiers
        #T[i,j] est le nombre minimal de pas qu'il faut faire en partant de 
        # (i,j) pour aller à la cible en évitant les obstacles (mais sans autres robots)
        #A partir de ça on peut reconstituer les chemins
        self.dicFils = {}
        self.calculerChemin()
        
        self.r = 0.3
        self.a = 0.2
        self.d = 4
        self.poisson = 0.2
        
    def reset(self, T):
        self.p =self.depart
        self.dernier_pas =ZERO
        self.T = T
        self.dicFils = {}
        self.calculerChemin()
        
        
    def effacerChemin(self, obs):
        #print("effacerChemin")
        L = [fils for fils in self.dicFils[obs]]
        self.dicFils[obs] = []
        aRecalculer = []
        Tbefore = np.copy(self.T)
        while len(L) > 0:
            c = L.pop(0)
            aRecalculer.append(c)
            self.T[c] = -1
            L += [fils for fils in self.dicFils[c]]
            self.dicFils[c] = []
        
        
        Tmiddle = np.copy(self.T)
        probleme = self.recalculerChemin(aRecalculer)
        
        
        if False:#probleme:
            print("T before:")
            print(Tbefore)
            print("OBS=", obs)
            print("\nT middle:")
            print(Tmiddle)
            print("\nT after:")
            print(self.T)
            print("\n\n")
            time.sleep(1)
        
        
    def recalculerChemin(self, aRecalculer):
        L = copy(aRecalculer)
        vaEtreRefait = []
        self.T[self.cible]=0
        while len(L)>0 :
            c = L.pop(0)
            voisins = self.voisins(c)
            if len(voisins) > 0:
                indexMin = 0
                mini = INFINITY
                for i in range(0,len(voisins)):
                    if 0 <= self.T[voisins[i]] < mini:
                        indexMin = i
                        mini = self.T[voisins[i]]
                                
                if not(mini >= INFINITY):
                    newC = voisins[indexMin]       
                    if not(c in self.dicFils[newC]):
                        self.dicFils[newC].append(c)
                    self.T[c]=self.T[newC]+1
                    
                    if c in vaEtreRefait:
                        vaEtreRefait.remove(c)
                        
                    if len(vaEtreRefait) > 0:
                        for v in self.voisins(c):
                            if v in vaEtreRefait:
                                vaEtreRefait.remove(v)
                                L.append(v)
                else:
                    if not(c in vaEtreRefait):
                        vaEtreRefait.append(c)
                        self.T[c] = -1
                    else:
                        #print("hey")
                        pass
                    #print("hey", c)
                    #print("Robot " + str(self.index) + " bloqué")
            else:
                #print("Robot " + str(self.index) + " bloqué")
                pass
        return len(vaEtreRefait) > 0
            

        
    def ajouterObstacle(self, obs):
        self.T[obs] = OBS
        self.effacerChemin(obs)
        return
        
        for x in range(len(self.T)):
            for y in range(len(self.T[0])):
                if self.T[x][y] >= 0:
                    self.T[x][y] = -1
        
        self.dicFils = {}
        self.calculerChemin()
        
        
    def ajouterObstacles(self, liste_obs, recalculer=True):
        for obs in liste_obs:
            self.T[obs] = OBS
            #self.effacerChemin(obs)
        #return
        
        if recalculer:
            for x in range(len(self.T)):
                for y in range(len(self.T[0])):
                    if self.T[x][y] >= 0:
                        self.T[x][y] = -1
            
            self.dicFils = {}
            self.calculerChemin()
        
        
    def enleverObstacles(self, liste_obs, recalculer=True):
        for obs in liste_obs:
            self.T[obs] = -1
            #self.effacerChemin(obs)
        #return
        
        if recalculer:
            for x in range(len(self.T)):
                for y in range(len(self.T[0])):
                    if self.T[x][y] >= 0:
                        self.T[x][y] = -1
            
            self.dicFils = {}
            self.calculerChemin()
        
        
    def distance(self):
        '''En combien de pas le robot peut-il arriver à destination'''
        return self.T[self.p]
    
    def prochainspas(self):
        '''Quels sont les prochains pas qui permettent à notre robot d'avancer'''
        V = self.voisins(self.p)
        R = []
        for v in V :
            if self.T[v]==self.T[self.p]-1:
                R.append(v)
        return  R
    
    def params(self,r,a,d, poisson):
        self.r = r
        self.a = a
        self.d = d
        self.poisson = poisson
        
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
        
        
        decisions_possibles = [ (elt[0]-p[0],elt[1]-p[1]) for elt in self.voisins(p)] + [(0,0)]

        
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
                    if d<4:
                        cases_prises_proches.append((x,y))
                else:
                    sameDir.append((x,y))
        
        for c in cases_prises_proches :
            for d in decisions_possibles :
                if dist(c,d)==1:
                    poids_direction[index(d)]-=2.*REPULSION
                if dist(c,d)==2:
                    poids_direction[index(d)]-=1.*REPULSION
        
        for d in decisions_possibles :
            d_abs = (p[0]+d[0],p[1]+d[1])
            delta = self.T[p]-self.T[d_abs]
            
            if delta== 1 : delta = DETERMINISTE
            """
            if d in sameDir:
                delta += POISSON
            """
            poids_direction[index(d)] += delta
        
        interdits = []
        for i in reversed(range(len(poids_direction))) :
            a = directions[i]
            if (a not in decisions_possibles) or (a in cases_prises_proches) :
                poids_direction[i]=0
                interdits.append(i)
        
        mini = min(poids_direction)
        for i in range(len(poids_direction)):
            if i not in interdits :
                poids_direction[i] += ALEATOIRE -mini
        somme = sum([elt for elt in poids_direction])
        poids_direction=[elt/somme for elt in poids_direction]
        pas = random.choices(directions, weights=poids_direction, k=1)[0]
        self.p = (self.p[0]+pas[0],self.p[1]+pas[1])
        self.lastDir = directions_utils[index(pas)]
        return self.lastDir
        print("Erreur !")
        



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



def calculPriorites2(pilotes):
    pasEncoreArrives = [p.index for p in pilotes]
    dejaArrives = []
    obstacles = []
    groupes = []
    distance = 0
    
    while len(pasEncoreArrives) > 0:
        groupes.append([])
        [pilotes[index].enleverObstacles(obstacles, recalculer=False) for index in pasEncoreArrives]
        obstacles = [pilotes[index].cible for index in pasEncoreArrives] + [pilotes[index].p for index in dejaArrives]
        [pilotes[index].ajouterObstacles(obstacles, recalculer=True) for index in pasEncoreArrives]
        ajout = False
        
        for index in copy(pasEncoreArrives):
            if pilotes[index].T[pilotes[index].p] >= 0: #pilotes[index].p
                distance += pilotes[index].T[pilotes[index].p]
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
                
            


def trouverSolution(file, optimizeMakespan = True, maxMakespan = 200, maxDistance = 10000, timeMax = 10,
                    marge = 5, repulsionMoy=0.3, repulsionVariation=0.1,
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
    taille = 0
    for r in range(i.number_of_robots):
        taille = max(taille, max(i.start_of(r)))
        taille = max(taille, max(i.target_of(r)))
    taille +=1
    
    # On rajoute la marge
    tailleBox = taille
    taille += 2*marge
    print(taille)
    
    
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
    obstacles_decales = [ (a+marge,b+marge) for (a,b) in i.obstacles]
    T = np.zeros((taille,taille),dtype=int)-1
    for o in obstacles_decales :
        T[o] = OBS
        
    for r in range(i.number_of_robots):
        pilotes.append(Pilote(r, taille,(i.target_of(r)[0]+marge,i.target_of(r)[1]+marge) ,(i.start_of(r)[0]+marge,i.start_of(r)[1]+marge), copy(T)))
    
    print("Pilotes initialisés")
    print(calculPriorites2(pilotes))
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
        [ elt.reset(T.copy()) for elt in pilotes]
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
        
        while not(needReset) and (nbArrives < nbRobotsTotal) and ((optimizeMakespan and makespan<makespanMini) or (not(optimizeMakespan) and distance<distanceMini)):
            step = SolutionStep()
            stepIsEmpty = True # Booléen qui nous permettra de ne pas ajouter des steps inutiles
              
            nbArrives = len(pilotesArrives)
            
            # Anciennes positions des robots, à enlever au prochain pas
            caseToRemove = []
            cases_prises = [(elt.p[0], elt.p[1], Direction.WAIT) for elt in pilotes] # Cases inaccessibles
            
            random.shuffle(pilotesActifs)
            for monPilote in pilotesActifs:
                pp = None # Action du robot (WAIT, SOUTH, NORTH, WEST, EAST)
                if monPilote.p == monPilote.cible: 
                    nbArrives += 1
                    #print("Le robot "+str(r)+" est arrivé à destination")
                    pp = Direction.WAIT
                    pilotesArrives.append(monPilote)
                    pilotesActifs.remove(monPilote)
                    
                    for p in pilotes:
                        p.ajouterObstacle(monPilote.p)
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
        print(nbArrives)
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
    
    #print(nbArrives)
    
    
    
    
    
    
            
              
                

            




#"small_000_10x10_20_10.instance"
#"small_free_001_10x10_40_40.instance"
trouverSolution("small_001_10x10_40_30.instance", maxMakespan = 1000, optimizeMakespan = True,
                timeMax=60)