# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 23:49:52 2021

@author: josse
"""

import time
from random import randint
n= 100000

t0 = time.time()
for _ in range(n):
    l = [randint(0,10) for _ in range(20)]
    if 5 in l:
        l.remove(5)
        
print(time.time() - t0)


t0 = time.time()
for _ in range(n):
    l = [randint(0,10) for _ in range(20)]
    try:
        l.remove(5)
    except ValueError:
        pass
        
print(time.time() - t0)
