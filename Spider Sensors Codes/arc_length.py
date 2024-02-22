from sympy import diff
import math
from sympy import sin
from sympy import cos
from sympy import symbols
import numpy as np
import sympy as sy
import matplotlib.pyplot as plt
from scipy import integrate
from scipy.integrate import quad


#https://mathworld.wolfram.com/Hypocycloid.html
## derivation of hypocycloid function

#### naming spider spiral location and angles from 1 to 6 each spiral in each stage
#### location 1  angle from  to  follow the figure in the paper
#### location 2  angle from to 
#### location 3 angle from to 

###### largest spider spiral ###########
Ys_2_D= lambda Phi: (-2.5 * np.sin(Phi) - 2.5 * np.sin(5.0*Phi))
Xs_2_D= lambda Phi: (2.5 * np.cos(Phi) - 2.5 * np.cos(5.0*Phi))


### arc length in location 1 according to angles  and radials intersection #####
arc_l_1 = quad(lambda Phi: np.sqrt(Ys_2_D(Phi)**2 + Xs_2_D(Phi)**2), np.pi/3, 2* (np.pi/3))
print(arc_l_1) ## without stretch


### arc length in location 2 check paper #####
arc_l_2 = quad(lambda Phi: np.sqrt(Ys_2_D(Phi)**2 + Xs_2_D(Phi)**2), 2* (np.pi/3), 3* (np.pi/3))
print(arc_l_2) ##without stretch

### arc length in location 3 check paper #####
arc_l_3 = quad(lambda Phi: np.sqrt(Ys_2_D(Phi)**2 + Xs_2_D(Phi)**2), 3* (np.pi/3), 4* (np.pi/3))
print(arc_l_3) ##without stretch



######  second stage spider spiral#########

Xs_1_D =lambda Phi: (-1.25*np.sin(Phi) - 1.25*np.sin(5.0*Phi))
Ys_1_D =lambda Phi: (1.25*np.cos(Phi) - 1.25*np.cos(5.0*Phi))

#arc_m = quad(lambda Phi: np.sqrt(Xs_1_D(Phi)**2 + Ys_1_D(Phi)**2), 0, 1.02051)
#print('middle arc is', arc_1)

### arc length in location 1 according to angles  and radials intersection #####
arc_m_1 = quad(lambda Phi: np.sqrt(Ys_1_D(Phi)**2 + Xs_1_D(Phi)**2), np.pi/3, 2* (np.pi/3))
print(arc_m_1) ## without stretch


### arc length in location 2 check paper #####
arc_m_2 = quad(lambda Phi: np.sqrt(Ys_1_D(Phi)**2 + Xs_1_D(Phi)**2), 2* (np.pi/3), 3* (np.pi/3))
print(arc_m_2) ##without stretch

### arc length in location 3 check paper #####
arc_m_3 = quad(lambda Phi: np.sqrt(Ys_1_D(Phi)**2 + Xs_1_D(Phi)**2), 3* (np.pi/3), 4* (np.pi/3))
print(arc_m_3) ##without stretch

######  smallest stage spider spiral ########

Xs_0_D =lambda Phi: (-0.625*np.sin(Phi) - 0.625*np.sin(5.0*Phi))
Ys_0_D = lambda Phi: (0.625*np.cos(Phi) - 0.625*np.cos(5.0*Phi))

#arc_s= quad(lambda Phi: np.sqrt(Xs_0_D(Phi)**2 + Ys_0_D(Phi)**2), 0, 1.02051)
#print('smallest arc is', arc_0)


### arc length in location 1 according to angles  and radials intersection #####
arc_s_1 = quad(lambda Phi: np.sqrt(Ys_0_D(Phi)**2 + Xs_0_D(Phi)**2), np.pi/3, 2* (np.pi/3))
print(arc_s_1) ## without stretch

### arc length in location 2 check paper #####
arc_s_2 = quad(lambda Phi: np.sqrt(Ys_0_D(Phi)**2 + Xs_0_D(Phi)**2), 2* (np.pi/3), 3* (np.pi/3))
print(arc_s_2) ##without stretch

### arc length in location 3 check paper #####
arc_s_3 = quad(lambda Phi: np.sqrt(Ys_0_D(Phi)**2 + Xs_0_D(Phi)**2), 3* (np.pi/3), 4* (np.pi/3))
print(arc_s_3) ##without stretch