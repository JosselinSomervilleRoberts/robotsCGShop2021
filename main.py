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
    
    def __init__(self, n, cible,depart,obstacles):
        self.depart = depart
        self.p = depart #La position actuelle du robot
        self.cible = cible #La ou le robot doit aller
        self.n = n
        self.T = np.zeros((n,n),dtype=int)-1
        self.dernier_pas =ZERO
        for o in obstacles :
            self.T[o] = OBS
        #Le tableau T est un tableau d'entiers
        #T[i,j] est le nombre minimal de pas qu'il faut faire en partant de 
        # (i,j) pour aller à la cible en évitant les obstacles (mais sans autres robots)
        #A partir de ça on peut reconstituer les chemins
        L = [self.cible]
        self.T[self.cible]=0
        while len(L)>0 :
            c = L[0]
            V = self.voisins(c)
            for v in V :
                if self.T[v]==-1 :
                    L.append(v)
                    self.T[v]=self.T[c]+1
            L.pop(0)
        
        self.r = 0.3
        self.a = 0.2
        self.d = 4
    def reset(self):
        self.p =self.depart
        self.dernier_pas =ZERO
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
    def params(self,r,a,d):
        self.r = r
        self.a = a
        self.d = d
    def pas_probabiliste(self,cases_prises=[]):
        REPULSION = self.r
        ALEATOIRE = self.a
        DETERMINISTE = self.d
        '''Dans la stratégie *banc de poissons, on demande au robot de faire un pas
        Il renvoie le pas choisi en format tuple'''
        p = self.p
        cases_prises_relat = [ (r[0]-p[0],r[1]-p[1]) for r in cases_prises]
        poids_direction = [ [i,0.] for i in range(5)]
        decisions_possibles = [ (elt[0]-p[0],elt[1]-p[1]) for elt in self.voisins(p)] + [(0,0)]
        
        cases_prises_proches = []
        for r in cases_prises_relat :
            if r!=(0,0) :
                d = abs(r[0])+abs(r[1])
                if d<4 :
                    cases_prises_proches.append(r)
        
        for c in cases_prises_proches :
            for d in decisions_possibles :
                if dist(c,d)==1:
                    poids_direction[index(d)][1]-=2.*REPULSION
                if dist(c,d)==2:
                    poids_direction[index(d)][1]-=1.*REPULSION
        
        for d in decisions_possibles :
            d_abs = (p[0]+d[0],p[1]+d[1])
            delta = self.T[p]-self.T[d_abs]
            
            if delta== 1 : delta = DETERMINISTE
            poids_direction[index(d)][1] += delta
        
        for i in reversed(range(len(poids_direction))) :
            a = directions[i]
            if (a not in decisions_possibles) or (a in cases_prises_proches) :
                poids_direction.pop(i)
        
        mini = min([elt[1] for elt in poids_direction])
        for i in range(len(poids_direction)):
            poids_direction[i][1] += ALEATOIRE -mini
        somme = sum([elt[1] for elt in poids_direction])
        tirage = somme*np.random.random()
        while len(poids_direction)>0:
            if poids_direction[0][1]>tirage:
                pas = directions[int(poids_direction[0][0])]
                self.p = (self.p[0]+pas[0],self.p[1]+pas[1])
                return directions_utils[poids_direction[0][0]]
            tirage -= poids_direction[0][1]
            poids_direction.pop(0)
        print("Erreur !")
        
marge = 5

idb = InstanceDatabase("datasets.zip")

i= idb["small_001_10x10_40_30"]
#i= idb["medium_free_013_50x50_50_1250"]
taille = 0
for r in range(i.number_of_robots):
    print("Robot", r, "starts at", i.start_of(r)," and has to go to ", i.target_of(r))
    taille = max(taille, max(i.start_of(r)))
    taille = max(taille, max(i.target_of(r)))
taille +=1
print(taille)

taille+=2*marge

for o in i.obstacles:
    print(o, "is blocked")
        
print(i.number_of_robots)





solution = None 

pilotes = []
obstacles_decales = [ (a+marge,b+marge) for (a,b) in i.obstacles]
for r in range(i.number_of_robots):
        pilotes.append(Pilote(taille,(i.target_of(r)[0]+marge,i.target_of(r)[1]+marge) ,(i.start_of(r)[0]+marge,i.start_of(r)[1]+marge),obstacles_decales))
print("Pilotes initialisés")
mini = 200
b = 0
t = time.time()
while True :
    solution = Solution(i)
    a = 0
    makespan = 0
    [ elt.reset() for elt in pilotes]
    pr,pa,pd = 0.3*(0.8+0.4*np.random.random()), 0.2*(0.8+0.4*np.random.random()) ,30*(1+0.4*np.random.random())
    [elt.params(pr,pa,pd) for elt in pilotes]
    
    while a<i.number_of_robots and makespan<mini:
        step = SolutionStep()
        a = 0
        stop = True
        cases_prises = [elt.p for elt in pilotes]
        for r in range(i.number_of_robots) :
            monPilote = pilotes[r]
            if monPilote.p == monPilote.cible: 
                a+=1
                #print("Le robot "+str(r)+" est arrivé à destination")
                pp= Direction.WAIT
            else :
                pp = monPilote.pas_probabiliste(cases_prises)
                cases_prises.append(monPilote.p)
            step[r] = pp

    
        makespan +=1
        solution.add_step(step)
    b+=1
    if makespan <mini :
        print("Trouvé une solution au "+str(b)+"ième essai avec un makespan de "+str(solution.makespan))
        #print("Makespan:", solution.makespan)

        #print("r = "+str(pr)+", a ="+str(pa)+" d="+str(pd))
        mini = min(mini,solution.makespan)
        '''
        with SolutionZipWriter("out"+str(solution.makespan)+".zip") as szw:
            szw.add_solution(solution)
        '''
        
        try:
            validate(solution)
            print("solution validée par cgshop2021_pyutils.validate")
        except ZipReaderError as zre:
            print("Bad Zip:", zre)
        except InvalidSolutionError as ise:
            print("Bad Solution:", ise)
        except SolutionEncodingError as see:
            print("Bad Solution File:", see)

            




    
'''

n = 5
cible = (2,3)
depart = (0,0)
obstacles = [(0,2),(1,2),(2,2),(3,2)]
monPilote = Pilote(n,cible,depart,obstacles)
print(monPilote.T.transpose())

while monPilote.distance()!=0:
    print("Le robot est en "+str(monPilote.p))
    monPilote.p = monPilote.prochainspas()[0]

print("Le robot est arrivé à destination")

'''