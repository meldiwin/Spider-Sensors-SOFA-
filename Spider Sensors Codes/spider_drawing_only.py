from sympy import diff
from sympy import sin
from sympy import cos
from sympy import symbols
import numpy as np
import sympy as sy
import matplotlib.pyplot as plt

#https://mathworld.wolfram.com/Hypocycloid.html
a_2= 3 #the largest radius in the spider structure is 3 mm 
#https://mathworld.wolfram.com/Hypocycloid.html, from the following
#reference we can deduce a/b = 6 (as our design has six spiral)

n= 6

b_2= a_2/n  ## b is the smaller radius 


### how to calculate phi
Phi= np.linspace(np.pi/3,7*(np.pi/3) , 20000)

### the largest spiral

Ys_2 = (a_2-b_2)*np.sin(Phi) - b_2*np.sin(Phi*((a_2-b_2)/b_2))
Xs_2 = (a_2-b_2)*np.cos(Phi) + b_2*np.cos(Phi*((a_2-b_2)/b_2))
#Xs_prime = diff(Xs, Phi)

##### second stage spiral############## 
a_1= 1.5  # the middle spiral structure's radius
b_1= a_1/n

Ys_1 = (a_1-b_1)*np.sin(Phi) - b_1*np.sin(Phi*((a_1-b_1)/b_1))
Xs_1 = (a_1-b_1)*np.cos(Phi) + b_1*np.cos(Phi*((a_1-b_1)/b_1))


##### the third spiral ###############

a_0= 0.75 ### radius of the smallest spider in theb structure
b_0= a_0/n

Ys_0 = (a_0-b_0)*np.sin(Phi) - b_0*np.sin(Phi*((a_0-b_0)/b_0))
Xs_0 = (a_0-b_0)*np.cos(Phi) + b_0*np.cos(Phi*((a_0-b_0)/b_0))

##### Radials with angle parameters ###########
Phi_0= np.pi/3
k_0= np.sin((n-1)*Phi_0)
l_0= np.sin(Phi_0)
m_0= np.cos((n-1)*Phi_0)
q_0= np.cos(Phi_0)

#### radial 0 with angle #######

xl_0 = np.linspace(-1.5,1.5,100)
yl_0 = ((k_0 - (n-1)*l_0) /(m_0 + (n-1)*q_0)) * xl_0

#### radial 1 with angle ######
xl_1 = np.linspace(1.5,-1.5,100)
yl_1 = ((k_0 - (n-1)*l_0) /(m_0 + (n-1)*q_0)) * xl_0


#### Horizontal  radial  parameters ######
k_1 = np.sin(5*np.pi)
l_1 = np.sin(np.pi)
m_1 = np.cos(5*np.pi)
q_1 = np.cos(np.pi)

#### horizontal radial ######

x_2 = np.linspace(-a_2,a_2,100)
y_2 = ((k_1 - (n-1)*l_1) /(m_1 + (n-1)*q_1)) * x_2

plt.plot(Xs_2,Ys_2,Xs_1,Ys_1,Xs_0,Ys_0,xl_0,yl_0, xl_1,yl_1,x_2,y_2)

plt.grid()
plt.show()











