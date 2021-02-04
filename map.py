# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 23:25:41 2021

@author: josse
"""

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


NOT_CALCULATED = -1
OBS = -2


class Case:
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.distance = NOT_CALCULATED
        self.fils = []
        self.parent = []
        self.obstaclesAreclaculer = []



class Map:
    
    def __init__(self, nx, ny):
        self.nx = nx
        self.ny = ny
        self.cible = None
        self.obstaclesPermanents = []
        self.resetMap()
        self.aEteCaclcule = False
        self.obstaclesAreclaculer = []
        
        
    def getValue(self, pos):
        return self.map[pos[0]][pos[1]].distance
    
    def setCible(self, posCible):
        self.cible = self.map[posCible[0]][posCible[1]]
        
    def resetMap(self):
        self.map = [[Case(x,y) for y in range(self.ny)] for x in range(self.nx)]
        self.ajouterObstacles(self.obstaclesPermanents)
        self.aEteCaclcule = False
        self.obstaclesAreclaculer = []
        
    def ajouterObstacles(self, obstacles):
        for obs in obstacles:
            (x, y) = obs
            self.map[x][y].distance = OBS
            if not(self.map[x][y]) in self.obstaclesAreclaculer:
                self.obstaclesAreclaculer.append(self.map[x][y])
            
    def enleverObstacles(self, obstacles):
        for obs in obstacles:
            (x, y) = obs
            self.map[x][y].distance = NOT_CALCULATED
            
    def ajouterObstaclesPermanents(self, obstacles):
        self.obstaclesPermanents += [obs for obs in obstacles]
        self.ajouterObstacles(obstacles)
        
    def getVoisins(self, node):
        L = []
        x, y = node.x, node.y
        if (x > 0 and self.map[x-1][y].distance != OBS) :
            L.append(self.map[x-1][y])
        if (x < self.nx - 1 and self.map[x+1][y].distance != OBS) :
            L.append(self.map[x+1][y])
        if (y > 0 and self.map[x][y-1].distance != OBS) :
            L.append(self.map[x][y-1])
        if (y < self.ny - 1 and self.map[x][y+1].distance != OBS) :
            L.append(self.map[x][y+1])
        return L
    
    def getVoisinsXY(self, pos):
        L = []
        (x, y) = pos
        if (x > 0 and self.map[x-1][y].distance != OBS) :
            L.append((x-1,y))
        if (x < self.nx - 1 and self.map[x+1][y].distance != OBS) :
            L.append((x+1,y))
        if (y > 0 and self.map[x][y-1].distance != OBS) :
            L.append((x,y-1))
        if (y < self.ny - 1 and self.map[x][y+1].distance != OBS) :
            L.append((x,y+1))
        return L
        
        
    def bfs(self, x, y, L=None):
        nbACalculer = 0
        distActuelle = 100000
        if L is None:
            cible = self.map[x][y]
            cible.distance = 0
            self.cible = cible
            L = {0: [cible]}
            distActuelle = 0
            nbACalculer = 1
        else:
            for dist in L.keys():
                nbACalculer += len(L[dist])
                if dist < distActuelle: distActuelle = dist
        
        
        while nbACalculer > 0:
            while distActuelle in L and len(L[distActuelle]) == 0:
                distActuelle+=1
            if distActuelle+1 not in L:
                L[distActuelle+1] = []
                
            current = L[distActuelle].pop(0)
            nbACalculer -= 1
            voisins = self.getVoisins(current)
            
            for v in voisins:
                if v.distance == NOT_CALCULATED:
                    v.distance = current.distance + 1
                    L[current.distance + 1].append(v)
                    nbACalculer += 1
                    v.parent.append(current)
                    current.fils.append(v)
                elif v.distance > current.distance + 1:
                    v.distance = current.distance + 1
                    L[current.distance + 1].append(v)
                    nbACalculer += 1
                    v.parent = [current]
        
        self.aEteCaclcule = True
                    
                    
                    
    def bfs_nouveaux_obstacles(self):
        # Si on a jamais calculé le chemin, on le fait normalement
        if not(self.aEteCaclcule):
            if not(self.cible is None):
                self.obstaclesAreclaculer = []
                self.bfs(self.cible.x, self.cible.y)
            else:
                raise Exception("BFS_nouveaux_obstacles appelé sans calcul préalable du BFS ni cible")
            return
        
        
        L = []
        
        # Pour chaque nouveau obstacle, on regarde ses fils et si ses fils n'avaient que lui comme parent alors il faut les recalculer
        # Pour recacluler on ajoute à la liste L
        # On enlève toutes les relations de parenté qui ne sont plus d'actualité
        L1 = self.obstaclesAreclaculer
        while len(L1) > 0:
            obs = L1.pop(0)
            for filsss in obs.fils:
                if obs in filsss.parent:
                    filsss.parent.remove(obs)
                    if len(filsss.parent) == 0:
                        L.append(filsss)
                        filsss.distance = NOT_CALCULATED
                        L1.append(filsss)
            obs.fils = []
            
            for parenttt in obs.parent:
                if obs in parenttt.fils:
                    parenttt.fils.remove(obs)
                    
            obs.parent = []
            
        nouveauL = {}
        while len(L) > 0:
            current = L.pop()
            voisins = self.getVoisins(current)
            
            voisinsPlusProches = []
            for v in voisins:
                if not(v.distance == NOT_CALCULATED):
                    if len(voisinsPlusProches) == 0:
                        voisinsPlusProches.append(v)
                    elif v.distance < voisinsPlusProches[0].distance:
                        voisinsPlusProches = [v]
                    elif v.distance == voisinsPlusProches[0].distance:
                        voisinsPlusProches.append(v)
                        
            # On a trouvé un voisin qui n'est ni un mur, ni une case non calculée
            if len(voisinsPlusProches) > 0: # On peut calculer le chemin
                current.parent = voisinsPlusProches
                current.distance = current.parent[0].distance +1
                for v in voisinsPlusProches:
                    if not(current in v.fils):
                        v.fils.append(current)
                
                aRevoir = False
                for v in voisins:
                    if (v.distance == NOT_CALCULATED or v.distance > current.distance +1) and not(v in L):
                        aRevoir = True
                
                if aRevoir:
                    if current.distance not in nouveauL: nouveauL[current.distance] = []
                    nouveauL[current.distance].append(current)
                
        #self.bfs(None, None, sorted(nouveauL, key=lambda x: x.distance))
        self.bfs(None, None, nouveauL)
        self.obstaclesAreclaculer = []
        
                        
    def show(self):
        long_max = 0
        
        for ligne in [[self.map[x][y] for x in range(self.nx)] for y in range(self.ny-1,-1,-1)]:
            max_ligne = max([len(str(case.distance)) for case in ligne])
            if max_ligne > long_max: long_max = max_ligne
            
        for ligne in [[self.map[x][y] for x in range(self.nx)] for y in range(self.ny-1,-1,-1)]:
            s = ""
            for case in ligne:
                if case.distance == OBS:
                    s +=  "■"*long_max + " "
                elif case.distance == NOT_CALCULATED:
                    s+= " "*(1+long_max)
                else:
                    s += str(case.distance) + " "*(1+long_max-len(str(case.distance)))
            print("|" + s + "|")
                        
                    
        
                    