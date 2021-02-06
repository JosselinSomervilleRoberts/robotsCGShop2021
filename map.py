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

def dist(t1,t2):
    '''La distance de la valeur absolue entre deux tuples'''
    return abs(t1[0]-t2[0])+abs(t1[1]-t2[1])


class Case:
    
    __slots__ = ('x', 'y', 'distance', 'fils', 'parent', 'estPenalise')
    
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.distance = NOT_CALCULATED
        self.fils = []
        self.parent = []
        self.estPenalise = False



class Map:
    
    __slots__ = ('map' ,'nx', 'ny', 'cible', 'obstaclesPermanents', 'aEteCaclcule', 'obstaclesAreclaculer','penalisation')
    
    def __init__(self, nx, ny):
        self.nx = nx
        self.ny = ny
        self.cible = None
        self.obstaclesPermanents = []
        self.resetMap()
        self.aEteCaclcule = False
        self.obstaclesAreclaculer = []
        self.penalisation = 3
        
        
    def setAEviter(self, aEviter):
        for c in aEviter:
            self.map[c[0]][c[1]].estPenalise = True
            
    def getShortestPath(self, posDepart):
        if not(self.aEteCaclcule):
            raise Exception("La map n\'a pas été calculée")
            
        path = [posDepart]
        current = self.map[posDepart[0]][posDepart[1]]
        
        if not(current.distance >= 0):
            raise Exception("Il n'y a pas de chemin valide")
            
        while current != self.cible:
            voisins = self.getVoisins(current)
            voisins = sorted(voisins, key=lambda v: v.distance + 10000*(v.distance==NOT_CALCULATED))
            current = voisins[0]
            path.append((current.x, current.y))
            
        return path
        
        
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
        if (x > 0 and self.map[x-1][y].distance >= 0) :
            L.append((x-1,y))
        if (x < self.nx - 1 and self.map[x+1][y].distance >= 0) :
            L.append((x+1,y))
        if (y > 0 and self.map[x][y-1].distance >= 0) :
            L.append((x,y-1))
        if (y < self.ny - 1 and self.map[x][y+1].distance >= 0) :
            L.append((x,y+1))
        return L
    
    
    def remettreAZero(self):
        for ligne in self.map:
            for case in ligne:
                if case.distance >= 0:
                    case.distance = NOT_CALCULATED
        self.aEteCaclcule = False
        
        
    def bfs(self, x, y, L=None, distmax=10000):
        nbACalculer = 0
        distActuelle = 100000
        if L is None:
            if not(self.cible is None) and self.cible != self.map[x][y]:
                self.remettreAZero()
                
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
            while (distActuelle not in L) or (distActuelle in L and len(L[distActuelle]) == 0):
                distActuelle+=1
                
            # On récupère la case actuelle et ses voisins
            current = L[distActuelle].pop(0)
            nbACalculer -= 1
            voisins = self.getVoisins(current)
            
            for v in voisins:
                # Distance jusqu'a v si on passe par current
                distance_v = current.distance + 1 + self.penalisation*(v.estPenalise or current.estPenalise)
                ajouterChemin = False
                
                # Si jamais on a jamais calculé la case, on ajoute le chemin
                if v.distance == NOT_CALCULATED:
                    ajouterChemin = True
                    
                # Si on a déja calculé la case et que l'on trouve la même distance -> cela fait un parent de plus
                elif v.distance == distance_v:
                    v.parent.append(current)
                    current.fils.append(v)
                
                # Si on a déja calculé la case mais que l'on trouve une distance plus courte
                elif v.distance > distance_v:
                    # On supprime les parents précédents
                    for parent in v.parent:
                        if v in parent.fils:
                            parent.fils.remove(v)
                    ajouterChemin = True
                    
                if ajouterChemin:
                    v.distance = distance_v
                    v.parent = [current]
                    current.fils.append(v)
                    
                    # Si la case n'est pas trop loin on l'ajoute pour calculer ses voisins
                    if v.distance < distmax:
                        if not(v.distance) in L: L[v.distance] = []
                        L[v.distance].append(v)
                        nbACalculer += 1
        
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
        elif not(self.cible is None) and len(self.obstaclesAreclaculer) > self.nx*self.ny/10.0:
            self.remettreAZero()
            self.bfs(self.cible[0], self.cible[1])
        
        
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
            
        # On met a jour ce que l'on peut
        # nouveauL correspond à ce qui repassera par le BFS classique
        nouveauL = {}
        while len(L) > 0:
            # On récupère la case actuelle et ses voisins
            current = L.pop(0)
            voisins = self.getVoisins(current)
            
            # On cherche les voisins qui ont déja été calculé et qui ont la distance la plus courte
            # Il faut prendre en compte si les cases sont pénalisées
            voisinsPlusProches = []
            bestDist = None # Correspond à la distance de current en passant par le voisin le plus proche
            for v in voisins:
                # Si on a déja calculé la case
                if not(v.distance == NOT_CALCULATED):
                    distance_v = v.distance + 1 + self.penalisation * (v.estPenalise or current.estPenalise)
                    
                    # Si c'est le premier voisin, pas besoin de regarde sa distance, on le prends
                    if len(voisinsPlusProches) == 0:
                        voisinsPlusProches.append(v)
                        bestDist = distance_v
                    elif distance_v < bestDist: # On a trouvé plus court
                        voisinsPlusProches = [v]
                    elif distance_v == bestDist: # On a trouvé aussi bien (cela fera un parent en plus)
                        voisinsPlusProches.append(v)
                        
            # On a trouvé un voisin qui n'est ni un mur, ni une case non calculée
            if len(voisinsPlusProches) > 0: # On peut calculer le chemin
                current.parent = voisinsPlusProches
                current.distance = bestDist
                
                # On ajoute les relations de fils
                for v in voisinsPlusProches:
                    if not(current in v.fils):
                        v.fils.append(current)
                
                # On regarde à présent si on ne peut pas réduire la distance d'un voisin en passant par current d'abord
                aRevoir = False
                for v in voisins:
                    if (v.distance == NOT_CALCULATED or v.distance > current.distance + 1 + self.penalisation * (v.estPenalise or current.estPenalise)) and not(v in L):
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
                        
                    
        
                    