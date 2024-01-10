9# -*- coding: utf-8 -*-

import Sofa

from scipy.linalg import solve
import numpy as np


import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

################################ Paramters ##################################
R_radial = 3205  #one 
R_spiral_1  = 1731  # one 
R_spiral_2  = 1124  # one 
R_spiral_3  = 560.49 # one 


Q_1 = 2.7463E+03
Q_2 = -5.2908E+01
Q_3 = 1.0512E+01

S_1 = 6.9175E+03
S_2 = -5.7424E+02
S_3 = 2.0642E+01
S_4 = -2.4400E-01


# to add elements like Node or objects
import Sofa.Core
root = Sofa.Core.Node()
import math 
import numpy as np
from scipy import signal

import os


path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'


class SpiderController(Sofa.Core.Controller):

	def __init__(self, *args, **kwargs):
		Sofa.Core.Controller.__init__(self,*args, **kwargs) #needed
		self.time = 0.0
		self.node = kwargs['node']
		self.pos_matrix = kwargs['pos_matrix']
		self.pos_three_stage = kwargs['pos_three_stage']
		

            	
	def onAnimateBeginEvent(self,event):
            self.time = self.node.time.value
            self.pos_matrix= self.node.matrix.l_matrix.position.value
            self.pos_three_stage= self.node.matrix.three_stage.l_three_stage.position.value



            ramp_time = 5  # Time for each ramp (up and down)
            forces = np.piecewise(self.time , [self.time  < ramp_time, (ramp_time <= self.time ) & (self.time  <= 2*ramp_time), self.time  > 2*ramp_time],
                     [lambda t: 80000 * t, lambda t: 400000 - 80000 * (t - ramp_time), 0])

            self.node.matrix.FF.force.value = [0,forces,0]
            
                     

#*********************** Radials    ************************#

              ###### one ####
            
            one = self.pos_three_stage[2345][1] - self.pos_three_stage[2172][1]
            #print("one is :", one)
            
            epsilon_1 = ((one - 4.411502838134496)/4.411502838134496)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r1= 471.295558183/2711.890962292 *( 40.9786262984012 * epsilon_1 + 2711.890962292 )
            #print(r1)
            
        
            
             ###### two ####
            
            two = self.pos_three_stage[2613][1] - self.pos_three_stage[2347][1]
            #print("two is :", two)
            
            epsilon_2 = ((two - 4.049495697021499)/4.049495697021499)*100
            
            #print(epsilon_2 * 100)
            
            r2= 432.621127927/2711.890962292 *( 40.9786262984012 * epsilon_2 + 2711.890962292 )
            #print(r2)
                   
             ###### three ####
            
            three = self.pos_three_stage[2643][1] - self.pos_three_stage[2615][1]
            #print("three is :", three)
            
            epsilon_3 = ((three - 3.969497680661007)/3.969497680661007)*100
            
            #print(epsilon_3)
            
            r3= 424.074670186/2711.890962292 *( 40.9786262984012 * epsilon_3 + 2711.890962292 )
            #print(r3)
            
                                    
             ###### four ####
            
            four = self.pos_three_stage[2524][1] - self.pos_three_stage[2649][1]
            #print("four is :", four)
            epsilon_4 = ((four - 3.969501495365023)/3.969501495365023)*100
            #print(epsilon_4)
            
            r4 = 424.075077471/2711.890962292 *( 40.9786262984012 * epsilon_4 + 2711.890962292 )
            #print(r4)
            
            
                                                
             ###### five ####
            
            five = self.pos_three_stage[2388][1] - self.pos_three_stage[2521][1]
            #print("five is :", five)
            epsilon_5 = ((five - 4.6425056457500204)/4.6425056457500204)*100
            #print(epsilon_5)
            
            r5 = 495.974355487/2711.890962292 *( 40.9786262984012 * epsilon_5 + 2711.890962292 )
            #print(r5)
                                                            
             ###### six ####
            
            six = self.pos_three_stage[2259][1] - self.pos_three_stage[2385][1]
            #print("six is :", six)
            
            epsilon_6 = ((six - 3.6990013122550494)/3.6990013122550494)*100
            #print(epsilon_6)
            
            r6 = 395.176641369/2711.890962292 *( 40.9786262984012 * epsilon_6 + 2711.890962292 )
            
            #print(r6)       
            
                                                                        
             ###### seven ####
            
            seven = self.pos_three_stage[2339][0] - self.pos_three_stage[2146][0]
            #print("seven is :", seven)
            epsilon_7 = ((seven - 3.7379989624019956)/3.7379989624019956)*100
            
            #print(epsilon_7)
            
            r7 = 399.342889501/2711.890962292 *( 40.9786262984012 * epsilon_7 + 2711.890962292 )
            
            #print(r7)   
            
                                                                                    
             ###### eight ####
            
            eight = self.pos_three_stage[2590][0] - self.pos_three_stage[2354][0]
            #print("eight is :", eight)
            
            epsilon_8 = ((eight - 4.694999694824986)/4.694999694824986)*100
            
            #print(epsilon_8)
            
            r8 = 501.582467715/2711.890962292 *( 40.9786262984012 * epsilon_8 + 2711.890962292 )
            
            #print(r8)
                                                                                                
             ###### nine ####
            
            nine = self.pos_three_stage[2659][0] - self.pos_three_stage[2596][0]
            #print("nine is :", nine)
            
            epsilon_9 = ((nine - 4.01600265503)/4.01600265503)*100
            
            #print(epsilon_9)
            
            r9 = 429.042950599/2711.890962292 *( 40.9786262984012 * epsilon_9 + 2711.890962292 )
            #print(r9)
            
            
            ###### ten ####
            
            ten = self.pos_three_stage[2563][0] - self.pos_three_stage[2661][0]
            #print("ten is :", ten)
            
            epsilon_10 = ((ten - 4.013999938963998)/4.013999938963998)*100
            
            #print(epsilon_10)
            
            r10 = 428.828993778/2711.890962292 *( 40.9786262984012 * epsilon_10 + 2711.890962292 )
            #print(r10)
            
            
            ###### eleven ####
            
            eleven = self.pos_three_stage[2396][0] - self.pos_three_stage[2530][0]
            #print("eleven is :", eleven)
            
            
            epsilon_11 = ((eleven - 4.093997955322003)/4.093997955322003)*100
            #print(epsilon_11)
            
            r11 = 437.375448495 /2711.890962292 *( 40.9786262984012 * epsilon_11 + 2711.890962292 )
            
            #print(r11)
            
            ###### twelve ####
            
            twelve = self.pos_three_stage[2232][0] - self.pos_three_stage[2392][0]
            #print("twelve is :", twelve)
            
            epsilon_12 = ((twelve - 4.4599990844730115 )/ 4.4599990844730115)*100
            #print(epsilon_12)
            
            
            r12 = 476.476568876/2711.890962292 *( 40.9786262984012 * epsilon_12 + 2711.890962292 )
            
            #print(r12)

            
    ################################# Spirals ##############################################
            
               ######### thirteen ###########
            thirteen = self.pos_three_stage[8][1] - self.pos_three_stage[20][1]
            #print("thirteen is :", thirteen)
            
            ########### segment 1 #####################
            
            thirteen_1 = self.pos_three_stage[2164][1] - self.pos_three_stage[20][1]
            
            #print("thirteen_1 is :", thirteen_1)
            
            epsilon_13_1 = ((thirteen_1 - 2.934501647948508 )/ 2.934501647948508)*100
    
            r13_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_13_1 + 2711.890962292 )
            
            
            ########### segment 2 #####################
            thirteen_2 = self.pos_three_stage[110][1] - self.pos_three_stage[2164][1]
            
            #print("thirteen_2 is :", thirteen_2)
            
            epsilon_13_2 = ((thirteen_2 - 2.7385025024414773 )/ 2.7385025024414773)*100
            r13_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_13_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            
            thirteen_3 = self.pos_three_stage[920][1] - self.pos_three_stage[110][1]
            
            #print("thirteen_3 is :", thirteen_3)
            
            epsilon_13_3 = ((thirteen_3 - 2.953498840332145)/ 2.953498840332145)*100
            r13_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_13_3 + 2711.890962292 )
            
            
            ########### segment 4 #####################
            
            thirteen_4 = self.pos_three_stage[115][1] - self.pos_three_stage[920][1]
            
            #print("thirteen_4 is :", thirteen_4)
            
            epsilon_13_4 = ((thirteen_4 - 2.750495910644858 )/ 2.750495910644858)*100
            
            r13_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_13_4 + 2711.890962292 )
            
            
            ########### segment 5 #####################
            
            thirteen_5 = self.pos_three_stage[8][1] - self.pos_three_stage[115][1]
            
            #print("thirteen_5 is :", thirteen_5)
            
            epsilon_13_5 = ((thirteen_5 - 2.958999633783975 )/ 2.958999633783975)*100
            r13_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_13_5 + 2711.890962292 )
            
            r13 = r13_1 + r13_2 + r13_3 + r13_4 + r13_5 
            
            
                ######### fourteen ###########
            fourteen = self.pos_three_stage[119][1] - self.pos_three_stage[104][1]
            
            ########### segment 1 #####################
            fourteen_1 = self.pos_three_stage[129][1] - self.pos_three_stage[104][1]
           
            epsilon_14_1 = ((fourteen_1 - 1.7959976196279968)/ 1.7959976196279968)*100            
            r14_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_14_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            fourteen_2 = self.pos_three_stage[2372][1] - self.pos_three_stage[129][1]
            
            #print("fourteen_2 is :", fourteen_2)
            
            epsilon_14_2 = ((fourteen_2 - 1.9025001525885301 )/ 1.9025001525885301 )*100            
            r14_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_14_2 + 2711.890962292 )
            
            ########### segment 3 #####################
            fourteen_3 = self.pos_three_stage[3649][1] - self.pos_three_stage[2372][1]
            #print("fourteen_3 is :", fourteen_3)
            
            epsilon_14_3 = ((fourteen_3 - 1.8752498626710121 )/ 1.8752498626710121)*100            
            r14_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_14_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            fourteen_4 = self.pos_three_stage[530][1] - self.pos_three_stage[3649][1]
            #print("fourteen_4 is :", fourteen_4)
            
            epsilon_14_4 = ((fourteen_4 - 1.8102474212644637 )/1.8102474212644637)*100            
            r14_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_14_4 + 2711.890962292 )
            
                        
            ########### segment 5 #####################
            fourteen_5 = self.pos_three_stage[119][1] - self.pos_three_stage[530][1]
            #print("fourteen_5 is :", fourteen_5)
            
            epsilon_14_5 = ((fourteen_5 - 1.9319992065380234 )/ 1.9319992065380234)*100            
            r14_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_14_5 + 2711.890962292 )
            
            r14 = r14_1 + r14_2 + r14_3 + r14_4 + r14_5

        
            ######### fiveteen ###########
            fiveteen = self.pos_three_stage[295][1] - self.pos_three_stage[301][1]
            #print("fiveteen is :", fiveteen)
            
             ########### segment 1 #####################
            fiveteen_1 = self.pos_three_stage[300][1] - self.pos_three_stage[301][1]
            #print("fiveteen_1 is :", fiveteen_1)
            
            epsilon_15_1 = ((fiveteen_1 - 0.8990020751959662 )/ 0.8990020751959662)*100
            r15_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_15_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            fiveteen_2 = self.pos_three_stage[1230][1] - self.pos_three_stage[300][1]
            #print("fiveteen_2 is :", fiveteen_2)
            
            epsilon_15_2 = ((fiveteen_2 - 0.9319992065425424)/ 0.9319992065425424)*100
            r15_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_15_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            fiveteen_3 = self.pos_three_stage[2599][1] - self.pos_three_stage[1230][1]
            #print("fiveteen_3 is :", fiveteen_3)
            
            epsilon_15_3 = ((fiveteen_3 - 1.0196256637574095 )/1.0196256637574095)*100
            r15_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_15_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            fiveteen_4 = self.pos_three_stage[385][1] - self.pos_three_stage[2599][1]
            #print("fiveteen_4 is :", fiveteen_4)
            
            epsilon_15_4 = ((fiveteen_4 - 0.7213716506950476)/ 0.7213716506950476)*100
            r15_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_15_4 + 2711.890962292 )
            
            ########### segment 5 #####################
            fiveteen_5 = self.pos_three_stage[295][1] - self.pos_three_stage[385][1]
            #print("fiveteen_5 is :", fiveteen_5)
            
            epsilon_15_5 = ((fiveteen_5 -  1.0699996948200123 )/  1.0699996948200123)*100
            
            r15_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_15_5 + 2711.890962292 )
            
            r15 = r15_1 + r15_2 + r15_3 + r15_4 + r15_5
            
      
            
            ######### sixteen ###########
            sixteen = self.pos_three_stage[33][0] - self.pos_three_stage[21][0]
            #print("sixteen is :", sixteen)
            
            ########### segment 1 #####################
            sixteen_1 = self.pos_three_stage[827][0] - self.pos_three_stage[21][0]
            #print("sixteen_1 is :", sixteen_1)
            
            epsilon_16_1 = ((sixteen_1 - 2.9815025329589915 )/2.9815025329589915)*100
            r16_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_16_1 + 2711.890962292 )

            
            ########### segment 2 #####################
            sixteen_2 = self.pos_three_stage[3568][0] - self.pos_three_stage[827][0]
            #print("sixteen_2 is :", sixteen_2)
            
            epsilon_16_2 = ((sixteen_2 - 2.801248550415501 )/ 2.801248550415501)*100
            r16_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_16_2 + 2711.890962292 )


            ########### segment 3 #####################
            sixteen_3 = self.pos_three_stage[3367][0] - self.pos_three_stage[3568][0]
            #print("sixteen_3 is :", sixteen_3)
            
            epsilon_16_3 = ((sixteen_3 - 2.995748519897049 )/2.995748519897049)*100
            F1= Q_1 + Q_2 *  epsilon_16_3 + Q_3 *  epsilon_16_3 **2 
	          F2= S_1 + S_2 *  epsilon_16_3 + S_3 *  epsilon_16_3 **2  + S_4 *  epsilon_16_3 **3
            r16_3 = R_spiral_1/Q_1 * (1/(F1 **20) + 1/(F2 **20)) **(-0.05)
            
            
            
            ########### segment 4 #####################
            sixteen_4 = self.pos_three_stage[6046][0] - self.pos_three_stage[3367][0]
            #print("sixteen_4 is :", sixteen_4)
            
                        
            epsilon_16_4 = ((sixteen_4 - 2.9347515106204582 )/ 2.9347515106204582)*100
            r16_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_16_4 + 2711.890962292 )

            
            ########### segment 4 #####################
            sixteen_5 = self.pos_three_stage[33][0] - self.pos_three_stage[6046][0]
            #print("sixteen_5 is :", sixteen_5)
            
                        
            epsilon_16_5 = ((sixteen_5 - 2.7827510833740092)/ 2.7827510833740092)*100
            r16_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_16_5 + 2711.890962292 )
            
            r16 =  r16_1 +  r16_2 +  r16_3 +  r16_4 +  r16_5
            

            
            ######### seventeen ###########
            seventeen = self.pos_three_stage[182][0] - self.pos_three_stage[196][0]
            #print("seventeen is :", seventeen)
            
            ########### segment 1 #####################
            seventeen_1 = self.pos_three_stage[1012][0] - self.pos_three_stage[196][0]
            #print("seventeen_1 is :", seventeen_1)
                 
            epsilon_17_1 = ((seventeen_1 -2.040498733519996 )/ 2.040498733519996)*100
            r17_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_17_1 + 2711.890962292 )
            
            
            ########### segment 2 #####################
            seventeen_2 = self.pos_three_stage[2466][0] - self.pos_three_stage[1012][0]
            #print("seventeen_2 is :", seventeen_2)
            
            epsilon_17_2 = ((seventeen_2 -   1.8907163759048302 )/  1.8907163759048302)*100
            r17_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_17_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            seventeen_3 = self.pos_three_stage[3487][0] - self.pos_three_stage[2466][0]
            #print("seventeen_3 is :", seventeen_3)
            
                 
            epsilon_17_3 = ((seventeen_3 -  2.0415336469834244 )/2.0415336469834244)*100
            r17_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_17_3 + 2711.890962292 )
            
            
            ########### segment 4 #####################
            seventeen_4 = self.pos_three_stage[4238][0] - self.pos_three_stage[3487][0]
            #print("seventeen_4 is :", seventeen_4)
            
                 
            epsilon_17_4 = ((seventeen_4 - 1.8332509994507546 )/ 1.8332509994507546)*100
            r17_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_17_4 + 2711.890962292 )
            
            
            ########### segment 5 #####################
            seventeen_5 = self.pos_three_stage[182][0] - self.pos_three_stage[4238][0]
            #print("seventeen_5 is :", seventeen_5)
            
            epsilon_17_5 = ((seventeen_5 -  1.9749984741209943)/ 1.9749984741209943)*100
            r17_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_17_5 + 2711.890962292 )
            
            r17 = r17_1 + r17_2 + r17_3 + r17_4 + r17_5
            
            
        
                        
            ######### eighteen ###########
            eighteen = self.pos_three_stage[353][0] - self.pos_three_stage[347][0]
            #print("eighteen is :", eighteen)
            
                        
            ########### segment 1 #####################
            eighteen_1 = self.pos_three_stage[4158][0] - self.pos_three_stage[347][0]
            #print("eighteen_1 is :", eighteen_1) 
            
            epsilon_18_1 = ((eighteen_1 - 0.9567489624021803 )/ 0.9567489624021803)*100
            
            r18_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_18_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            eighteen_2 = self.pos_three_stage[5645][0] - self.pos_three_stage[4158][0]
            #print("eighteen_2 is :", eighteen_2) 
            
            epsilon_18_2 = ((eighteen_2 - 0.8465013504033365 )/ 0.8465013504033365)*100
            
            r18_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_18_2 + 2711.890962292 )
            
            ########### segment 3 #####################
            eighteen_3 = self.pos_three_stage[1184][0] - self.pos_three_stage[5645][0]
            #print("eighteen_3 is :", eighteen_3)
            
            epsilon_18_3 = ((eighteen_3 - 0.9457483291624911 )/ 0.9457483291624911)*100
            
            r18_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_18_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            eighteen_4 = self.pos_three_stage[4146][0] - self.pos_three_stage[1184][0]
            #print("eighteen_4 is :", eighteen_4) 
            
            epsilon_18_4 = ((eighteen_4 - 0.9905014038088495 )/0.9905014038088495)*100
            
            r18_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_18_4 + 2711.890962292 )
            
            ########### segment 4 #####################
            eighteen_5 = self.pos_three_stage[353][0] - self.pos_three_stage[4146][0]
            #print("eighteen_5 is :", eighteen_5) 
            
            epsilon_18_5 = ((eighteen_5 -  0.956499099731154 )/ 0.956499099731154)*100
            
            r18_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_18_5 + 2711.890962292 )
            
            r18 = r18_1 + r18_2 + r18_3 + r18_4 + r18_5
            
                                    
            ######### nineteen ###########
            nineteen = self.pos_three_stage[73][1] - self.pos_three_stage[7][1]
            #print("nineteen is :", nineteen)
            ########### segment 1 #####################
            nineteen_1 = self.pos_three_stage[5498][1] - self.pos_three_stage[7][1]
            #print("nineteen_1 is :", nineteen_1) 
                                    
            epsilon_19_1 = (( nineteen_1  - 2.500249862675105)/ 2.500249862675105)*100
            r19_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_19_1 + 2711.890962292 )
         
            ########### segment 2 #####################
            nineteen_2 = self.pos_three_stage[2][1] - self.pos_three_stage[5498][1]
            #print("nineteen_2 is :", nineteen_2) 
                                    
            epsilon_19_2 = (( nineteen_2  - 2.8717517852748955)/ 2.8717517852748955)*100
            r19_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_19_2 + 2711.890962292 )
        
            ########### segment 3 #####################
            nineteen_3 = self.pos_three_stage[5973][1] - self.pos_three_stage[908][1]
            #print("nineteen_3 is :", nineteen_3) 
            
            epsilon_19_3 = (( nineteen_3  -  2.8335018157919336)/  2.8335018157919336)*100
            r19_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_19_3 + 2711.890962292 )
         
            ########### segment 4 #####################
            nineteen_4 = self.pos_three_stage[2297][1] - self.pos_three_stage[5973][1]
            #print("nineteen_4 is :", nineteen_4) 
            
            epsilon_19_4 = (( nineteen_4  - 2.8584995269770843)/  2.8584995269770843)*100
            r19_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_19_4 + 2711.890962292 )
         
            ########### segment 4 #####################
            nineteen_5 = self.pos_three_stage[73][1] - self.pos_three_stage[2297][1]
            #print("nineteen_5 is :", nineteen_5) 
            
            epsilon_19_5 = (( nineteen_5  - 2.9345054626449922 )/ 2.9345054626449922)*100
            r19_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_19_5 + 2711.890962292 )
         
            r19 = r19_1 + r19_2 + r19_3 + r19_4 + r19_5
            
        

            
            ######### twenty ###########
            twenty = self.pos_three_stage[84][1] - self.pos_three_stage[96][1]
            #print("twenty is :", twenty)
            
            
            ########### segment 1 #####################
            twenty_1 = self.pos_three_stage[3733][1] - self.pos_three_stage[96][1]
            #print("twenty_1 is :", twenty_1) 
            
                
            epsilon_20_1 = (( twenty_1  - 1.7639999389652132)/1.7639999389652132)*100
            
            r20_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_20_1 + 2711.890962292 )
    
            ########### segment 2 #####################
            twenty_2 = self.pos_three_stage[5875][1] - self.pos_three_stage[503][1]
            #print("twenty_2 is :", twenty_2) 
            
                
            epsilon_20_2 = (( twenty_2  -1.7984438109037058)/ 1.7984438109037058)*100
            
            r20_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_20_2 + 2711.890962292 )
    
            ########### segment 3 #####################
            twenty_3 = self.pos_three_stage[2316][1] - self.pos_three_stage[5875][1]
            #print("twenty_3 is :", twenty_3) 
            
                
            epsilon_20_3 = (( twenty_3  - 1.8850622010613165 )/ 1.8850622010613165)*100
            
            r20_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_20_3 + 2711.890962292 )
    
            
                        
            ########### segment 4 #####################
            twenty_4 = self.pos_three_stage[1117][1] - self.pos_three_stage[2316][1]
            #print("twenty_4 is :", twenty_4) 
            
            epsilon_20_4 = (( twenty_4  - 1.8260002136246953 )/ 1.8260002136246953)*100
            
            r20_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_20_4 + 2711.890962292 )
    
            
            ########### segment 5 #####################
            twenty_5 = self.pos_three_stage[84][1] - self.pos_three_stage[1117][1]
            #print("twenty_5 is :", twenty_5) 
                
            epsilon_20_5 = (( twenty_5  - 1.8724975585902826)/ 1.8724975585902826)*100
            
            r20_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_20_5 + 2711.890962292 )
    
            
            r20 = r20_1 + r20_2 + r20_3 + r20_4 + r20_5
            
            
        
            ######### twentyone  ###########
            twentyone_ = self.pos_three_stage[276][1] - self.pos_three_stage[282][1]
            #print("twenty_1 is :", twenty_1)
            
                        
            ########### segment1  #################### 
            twentyone_1 = self.pos_three_stage[1102][1] - self.pos_three_stage[282][1]
            #print("twentyone_1 is :", twentyone_1)
            
            epsilon_21_1 = (( twentyone_1   - 0.945003509525975)/ 0.945003509525975)*100
            r21_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 *epsilon_21_1 + 2711.890962292 )
    
            
            ########### segment2  #################### 
            twentyone_2 = self.pos_three_stage[3150][1] - self.pos_three_stage[1102][1]
            #print("twentyone_2 is :", twentyone_2) 
            
            epsilon_21_2 = (( twentyone_2   - 0.9409980773890254)/ 0.9409980773890254)*100
            r21_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_21_2 + 2711.890962292 )
    
                         
            ########### segment3  #################### 
            twentyone_3 = self.pos_three_stage[278][1] - self.pos_three_stage[3150][1]
            #print("twentyone_3 is :", twentyone_3) 
            
            epsilon_21_3 = (( twentyone_3   - 0.9879989624049728)/ 0.9879989624049728)*100
            r21_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_21_3 + 2711.890962292 )
    
            
            ########### segment4  #################### 
            twentyone_4 = self.pos_three_stage[691][1] - self.pos_three_stage[278][1]
            #print("twentyone_4 is :", twentyone_4) 
            
            epsilon_21_4 = (( twentyone_4   - 0.8690032959000149)/ 0.8690032959000149)*100
            r21_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_21_4 + 2711.890962292 )
            
            ########### segment5 #################### 
            twentyone_5 = self.pos_three_stage[276][1] - self.pos_three_stage[691][1]
            #print("twentyone_5 is :", twentyone_5) 
            
            epsilon_21_5 = (( twentyone_5   - 0.8989944458000139)/ 0.8989944458000139)*100
            r21_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_21_5 + 2711.890962292 )
            
            r21 = r21_1 + r21_2 + r21_3 + r21_4 + r21_5
        
            
                         
            ######### twentytwo ###########
            twentytwo = self.pos_three_stage[60][0] - self.pos_three_stage[72][0]
            #print("twenty_2 is :", twenty_2)
           
            ########### segment1 #################### 
            twentytwo_1 = self.pos_three_stage[873][0] - self.pos_three_stage[72][0]
            #print("twentytwo_1  is :", twentytwo_1 ) 
            
            epsilon_22_1 = (( twentytwo_1   - 2.981502532958295)/ 2.981502532958295)*100
            r22_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_22_1 + 2711.890962292 )
            
            ########### segment2 #################### 
            twentytwo_2 = self.pos_three_stage[4374][0] - self.pos_three_stage[873][0]
            #print("twentytwo_2  is :", twentytwo_2 ) 
            
            epsilon_22_2 = (( twentytwo_2   - 2.801248550415778)/ 2.801248550415778)*100
            r22_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_22_2 + 2711.890962292 )
            
                      
            ########### segment3 #################### 
            twentytwo_3 = self.pos_three_stage[4755][0] - self.pos_three_stage[4374][0]
            #print("twentytwo_3  is :", twentytwo_3 ) 
            
            epsilon_22_3 = (( twentytwo_3   - 2.9957485198969636)/ 2.9957485198969636)*100
            r22_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_22_3 + 2711.890962292 )
            
            ########### segment4 #################### 
            twentytwo_4 = self.pos_three_stage[866][0] - self.pos_three_stage[4755][0]
            #print("twentytwo_4  is :", twentytwo_4) 
            
            epsilon_22_4 = (( twentytwo_4   - 2.736000061035277)/2.736000061035277)*100
            r22_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_22_4 + 2711.890962292 )  
            
                        
            ########### segment5 #################### 
            twentytwo_5 = self.pos_three_stage[60][0] - self.pos_three_stage[866][0]
            #print("twentytwo_5  is :", twentytwo_5) 
            
            epsilon_22_5 = (( twentytwo_5   - 2.9815025329596807)/ 2.9815025329596807)*100
            r22_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_22_5 + 2711.890962292 )                        
            
            r22 = r22_1 + r22_2 + r22_3 + r22_4 + r22_5
                                     
                                     
                                     
            ######### twentythree ###########
            twentythree = self.pos_three_stage[222][0] - self.pos_three_stage[208][0]
            #print("twentythree is :", twenty_3)
            
            ########### segment1 #################### 
            twentythree_1 = self.pos_three_stage[5176][0] - self.pos_three_stage[208][0]
            #print("twentythree_1  is :", twentythree_1) 
            
            epsilon_23_1 = (( twentythree_1   - 1.9342489242550158)/1.9342489242550158)*100
            r23_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_23_1 + 2711.890962292 )   
            
            ########### segment2 ################### 
            twentythree_2 = self.pos_three_stage[2504][0] - self.pos_three_stage[5176][0]
            #print("twentythree_2  is :", twentythree_2) 
            
            epsilon_23_2 = (( twentythree_2   - 1.9969661436740154)/ 1.9969661436740154)*100
            r23_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_23_2 + 2711.890962292 )   
            
            ########### segment3 ################### 
            twentythree_3 = self.pos_three_stage[5309][0] - self.pos_three_stage[2504][0]
            #print("twentythree_3  is :", twentythree_3) 
            
            epsilon_23_3 = (( twentythree_3   - 1.9910328187286268)/ 1.9910328187286268)*100
            r23_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_23_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentythree_4 = self.pos_three_stage[1040][0] - self.pos_three_stage[5309][0]
            #print("twentythree_4  is :", twentythree_4) 
            
            epsilon_23_4 = (( twentythree_4   - 1.883751869201859)/ 1.883751869201859)*100
            r23_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_23_4 + 2711.890962292 )
            
            
            ########### segment5 ################### 
            twentythree_5 = self.pos_three_stage[222][0] - self.pos_three_stage[1040][0]
            #print("twentythree_5  is :", twentythree_5) 
            
            epsilon_23_5 = (( twentythree_5   - 1.974998474120497)/ 1.974998474120497)*100
            r23_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_23_5 + 2711.890962292 )
            
            r23 = r23_1 + r23_2 + r23_3 + r23_4 + r23_5 
            
            
                                                 
            ######### twentyfour ###########
            twentyfour = self.pos_three_stage[315][0] - self.pos_three_stage[321][0]
            #print("twentyfour is :", twenty_4)
            
            ########### segment1 ################### 
            twentyfour_1 = self.pos_three_stage[364][0] - self.pos_three_stage[321][0]
            #print("twentyfour_1  is :", twentyfour_1) 
            
            epsilon_24_1 = (( twentyfour_1   - 0.9850006103519959)/ 0.9850006103519959)*100
            r24_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_24_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            twentyfour_2 = self.pos_three_stage[5195][0] - self.pos_three_stage[364][0]
            #print("twentyfour_2  is :", twentyfour_2) 
            
            epsilon_24_2 = (( twentyfour_2   - 0.8182497024533291)/ 0.8182497024533291)*100
            r24_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_24_2 + 2711.890962292 )
            
                        
            ########### segment3 ################### 
            twentyfour_3 = self.pos_three_stage[1147][0] - self.pos_three_stage[5195][0]
            #print("twentyfour_3  is :", twentyfour_3) 
            
            epsilon_24_3 = (( twentyfour_3   - 0.9457483291624911)/ 0.9457483291624911)*100
            r24_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_24_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentyfour_4 = self.pos_three_stage[2619][0] - self.pos_three_stage[1147][0]
            #print("twentyfour_4  is :", twentyfour_4) 
            
            epsilon_24_4 = (( twentyfour_4   - 1.0750007629396876)/ 1.0750007629396876)*100
            r24_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_24_4 + 2711.890962292 )
            
            ########### segment5 ################### 
            twentyfour_5 = self.pos_three_stage[315][0] - self.pos_three_stage[2619][0]
            #print("twentyfour_5  is :", twentyfour_5) 
            
            epsilon_24_5 = (( twentyfour_5   - 0.8719997406005007)/ 0.8719997406005007)*100
            r24_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_24_5 + 2711.890962292 )
            
            r24 = r24_1 + r24_2 + r24_3 + r24_4 + r24_5
         
            
            ######### twentyfive ###########
            twentyfive = self.pos_three_stage[118][1] - self.pos_three_stage[106][1]
            #print("twenty_5 is :", twenty_5)
            
            ########### segment1 ################### 
            twentyfive_1 = self.pos_three_stage[5005][1] - self.pos_three_stage[47][1]
            #print("twentyfive_1 is :", twentyfive_1) 
            
            epsilon_25_1 = (( twentyfive_1  - 2.8257503509586854)/ 2.8257503509586854)*100
            r25_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_25_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            twentyfive_2 = self.pos_three_stage[946][1] - self.pos_three_stage[5005][1]
            #print("twentyfive_2 is :", twentyfive_2) 
            
            epsilon_25_2 = (( twentyfive_2  - 2.8837490081753003)/ 2.8837490081753003)*100
            r25_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_25_2 + 2711.890962292 )
            
            ########### segment3 ################### 
            twentyfive_3 = self.pos_three_stage[5972][1] - self.pos_three_stage[946][1]
            #print("twentyfive_3 is :", twentyfive_3) 
            
            epsilon_25_3 = (( twentyfive_3  - 2.833501815793909)/ 2.833501815793909)*100
            r25_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_25_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentyfive_4 = self.pos_three_stage[2251][1] - self.pos_three_stage[5972][1]
            #print("twentyfive_4 is :", twentyfive_4)
            
            epsilon_25_4 = (( twentyfive_4  - 2.8584995269771127)/ 2.8584995269771127)*100
            r25_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_25_4 + 2711.890962292 )
            
                        
            ########### segment5 ################### 
            twentyfive_5 = self.pos_three_stage[59][1] - self.pos_three_stage[2251][1]
            #print("twentyfive_5 is :", twentyfive_5) 
            
            epsilon_25_5 = (( twentyfive_5  - 2.934505462644978)/ 2.934505462644978)*100
            r25_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_25_5 + 2711.890962292 )
            
            r25 = r25_1 + r25_2 + r25_3 + r25_4 + r25_5
            
            
            
            
            ######### twentysix ###########
            twentysix = self.pos_three_stage[294][1] - self.pos_three_stage[302][1]
            #print("twenty_6 is :", twenty_6)
            
            
            ########### segment1 ################### 
            twentysix_1 = self.pos_three_stage[245][1] - self.pos_three_stage[145][1]
            #print("twentysix_1 is :", twentysix_1) 
            
            epsilon_26_1 = ((  twentysix_1  - 2.0360031128000315)/ 2.0360031128000315)*100
            r26_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_26_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            twentysix_2 = self.pos_three_stage[2117][1] - self.pos_three_stage[245][1]
            #print("twentysix_2 is :", twentysix_2) 
            
            epsilon_26_2 = ((  twentysix_2  - 1.9912858140034047)/ 1.9912858140034047)*100
            r26_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_26_2 + 2711.890962292 )
            
            
            ########### segment3 ################### 
            twentysix_3 = self.pos_three_stage[6120][1] - self.pos_three_stage[2117][1]
            #print("twentysix_3 is :", twentysix_3) 
            
            epsilon_26_3 = ((  twentysix_3  - 1.884961591999371)/ 1.884961591999371)*100
            r26_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_26_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentysix_4 = self.pos_three_stage[1055][1] - self.pos_three_stage[6120][1]
            #print("twentysix_4 is :", twentysix_4) 
            
            epsilon_26_4 = ((  twentysix_4  - 2.0852508544923296)/ 2.0852508544923296)*100
            r26_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_26_4 + 2711.890962292 )
          
          
            ########### segment5 ################### 
            twentysix_5 = self.pos_three_stage[131][1] - self.pos_three_stage[1055][1]
            #print("twentysix_5 is :", twentysix_5) 
            
            epsilon_26_5 = ((  twentysix_5  - 2.0304985046348776)/ 2.0304985046348776)*100
            r26_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_26_5 + 2711.890962292 )
            
            r26 = r26_1 + r26_2 + r26_3 + r26_4 + r26_5
            
            
            ######### twentyseven ###########
            twentyseven = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twenty_7 is :", twenty_7)
            
            ########### segment1 ################### 
            twentyseven_1 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_1  is :", twentyseven_1 )
        
            epsilon_27_1 = (( twentyseven_1 - 2.0304985046348776)/ 2.0304985046348776)*100
            r27_1 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_27_1 + 2711.890962292 )
            
            
            ########### segment2 ################### 
            twentyseven_2 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_2 is :", twentyseven_2)
        
            epsilon_27_2 = (( twentyseven_2 - 2.0304985046348776)/ 2.0304985046348776)*100
            r27_2 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_27_2 + 2711.890962292 )
           
           
            ########### segment3 ################### 
            twentyseven_3 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_3 is :", twentyseven_3)
        
            epsilon_27_3 = (( twentyseven_3 - 2.0304985046348776)/ 2.0304985046348776)*100
            r27_3 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_27_3 + 2711.890962292 )
            
            
            ########### segment4 ################### 
            twentyseven_4 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_4 is :", twentyseven_4)
        
            epsilon_27_4 = (( twentyseven_4 - 2.0304985046348776)/ 2.0304985046348776)*100
            r27_4 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_27_4 + 2711.890962292 )
            
            
            ########### segment5 ################### 
            twentyseven_5 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_5 is :", twentyseven_5)
        
            epsilon_27_5 = (( twentyseven_5 - 2.0304985046348776)/ 2.0304985046348776)*100
            r27_5 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_27_5 + 2711.890962292 )
            
            r27 = r27_1 + r27_2 + r27_3 + r27_4 + r27_5
            
            
            ######### twentyeight ###########
            twentyeight = self.pos_three_stage[83][1] - self.pos_three_stage[97][1]
            #print("twenty_8 is :", twenty_8)
            
            ########### segment1 ################### 
            
            twentyeight_1 = self.pos_three_stage[2210][1] - self.pos_three_stage[34][1]
            #print("twentyeight_1 is :", twentyeight_1)
            
            epsilon_28_1 = ((twentyeight_1 - 2.934501647948494)/2.934501647948494)*100
            r28_1 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_28_1 + 2711.890962292 )
            
                        
            ########### segment2 ################### 
            
            twentyeight_2 = self.pos_three_stage[165][1] - self.pos_three_stage[2210][1]
            #print("twentyeight_2 is :", twentyeight_2)
            
            epsilon_28_2 = ((twentyeight_2 - 2.738502502441534)/2.738502502441534)*100
            r28_2 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_28_2 + 2711.890962292 )
            
            
            ########### segment3 ################### 
            
            twentyeight_3 = self.pos_three_stage[974][1] - self.pos_three_stage[165][1]
            #print("twentyeight_3 is :", twentyeight_3)
            
            epsilon_28_3 = ((twentyeight_3 - 2.953498840332145)/ 2.953498840332145)*100
            r28_3 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_28_3 + 2711.890962292 )
            
            ########### segment 4 ################### 
            
            twentyeight_4 = self.pos_three_stage[160][1] - self.pos_three_stage[974][1]
            #print("twentyeight_4 is :", twentyeight_4)
            
            epsilon_28_4 = ((twentyeight_4 - 2.7504959106448297)/ 2.7504959106448297)*100
            r28_4 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_28_4 + 2711.890962292 )
            
             ########### segment 5 ################### 
            
            twentyeight_5 = self.pos_three_stage[46][1] - self.pos_three_stage[160][1]
            #print("twentyeight_5 is :", twentyeight_5)
            
            epsilon_28_5 = ((twentyeight_5 - 2.9589996337839892)/ 2.9589996337839892)*100
            r28_5 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_28_5 + 2711.890962292 )
            
            r28 = r28_1 + r28_2 + r28_3 + r28_4 + r28_5
         
            
            
            ######### twentynine ###########
            twentynine = self.pos_three_stage[275][1] - self.pos_three_stage[283][1]
            #print("twenty_9 is :", twenty_9)
            
                        
            ######### segment 1 ###########
            twentynine_1 = self.pos_three_stage[1089][1] - self.pos_three_stage[170][1]
            #print("twentynine_1 is :", twentynine_1)
            
            epsilon_29_1 = ((twentynine_1 - 2.0315017700190197)/ 2.0315017700190197)*100
            r29_1 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_29_1 + 2711.890962292 )
            
                             
            ######### segment 2 ###########
            twentynine_2 = self.pos_three_stage[3984][1] - self.pos_three_stage[1089][1]
            #print("twentynine_2 is :", twentynine_2)
            
            epsilon_29_2 = ((twentynine_2 - 2.075000762939368)/ 2.075000762939368)*100
            r29_2 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_29_2 + 2711.890962292 )
            
            ######### segment 3 ###########
            twentynine_3 = self.pos_three_stage[3966][1] - self.pos_three_stage[3984][1]
            #print("twentynine_3 is :", twentynine_3)
            
            epsilon_29_3 = ((twentynine_3 - 1.9287509918219854)/ 1.9287509918219854)*100
            r29_3 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_29_3 + 2711.890962292 )
            
            ######### segment 4 ###########
            twentynine_4 = self.pos_three_stage[3955][1] - self.pos_three_stage[3966][1]
            #print("twentynine_4 is :", twentynine_4)
            
            epsilon_29_4 = ((twentynine_4 - 2.000497817992283)/ 2.000497817992283)*100
            r29_4 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_29_4 + 2711.890962292 )
            
            ######### segment 5 ###########
            twentynine_5 = self.pos_three_stage[156][1] - self.pos_three_stage[3955][1]
            #print("twentynine_5 is :", twentynine_5)
            
            epsilon_29_5 = ((twentynine_5 - 1.9932479858353531)/1.9932479858353531)*100
            r29_5 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_29_5 + 2711.890962292 )
            
            r29 = r29_1 + r29_2 + r29_3 + r29_4 + r29_5
            
            
            ######### thirty ###########
            thirty = self.pos_three_stage[377][1] - self.pos_three_stage[371][1]
            #print(" thirty is :",  thirty)
            
            ######### segment 1 ###########
           
            thirty_1 = self.pos_three_stage[656][1] - self.pos_three_stage[254][1]
            #print(" thirty_1 is :",  thirty_1)
            
            epsilon_30_1 = ((thirty_1 - 0.8990020751960088)/0.8990020751960088)*100
            r30_1 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_30_1 + 2711.890962292 )
            
            
            ######### segment 2 ###########
           
            thirty_2 = self.pos_three_stage[675][1] - self.pos_three_stage[656][1]
            #print(" thirty_2 is :",  thirty_2)
            
            epsilon_30_2 = ((thirty_2 - 0.8700027465820028)/0.8700027465820028)*100
            r30_2 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_30_2 + 2711.890962292 )
            
            ######### segment 3 ###########
           
            thirty_3 = self.pos_three_stage[2554][1] - self.pos_three_stage[675][1]
            #print(" thirty_3 is :",  thirty_3)
            
            epsilon_30_3 = ((thirty_3 - 1.081622123718006)/1.081622123718006)*100
            r30_3 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_30_3 + 2711.890962292 )
            
            ######### segment 4 ###########
           
            thirty_4 = self.pos_three_stage[1076][1] - self.pos_three_stage[2554][1]
            #print(" thirty_4 is :",  thirty_4)
            
            epsilon_30_4 = ((thirty_4 - 0.8468751907334848)/0.8468751907334848)*100
            r30_4 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_30_4 + 2711.890962292 )
            
        
            ######### segment 5 ###########
           
            thirty_5 = self.pos_three_stage[260][1] - self.pos_three_stage[1076][1]
            #print(" thirty_5 is :",  thirty_5)
            
            epsilon_30_5 = ((thirty_5 - 0.9444961547815325)/0.9444961547815325)*100
            r30_5 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_30_5 + 2711.890962292 )
            
            r30 = r30_1 + r30_2 + r30_3 + r30_4 + r30_5
            
            
            
            ######### radials  ###########
            
                        
            ######### thirty1 ###########
            thirtyone = self.pos_three_stage[209][1] - self.pos_three_stage[208][1]
            epsilon_31 = ((thirtyone - 4.411994934079999)/ 4.411994934079999)*100
            #print(epsilon_31)
            
            r31 = 471.348126605/2711.890962292 *( 40.9786262984012 * epsilon_31 + 2711.890962292 )
            

           
            ######### thirty2 ###########
            thirtytwo  = self.pos_three_stage[322][1] - self.pos_three_stage[321][1]
            epsilon_32 = ((thirtytwo  - 4.049003601069984 )/4.049003601069984 )*100
            #print(epsilon_32)
            
            r32 = 432.56855176/2711.890962292 *( 40.9786262984012 * epsilon_32 + 2711.890962292 )
            #print(r32) 
    
            
            ######### thirty3 ###########
            thirtythree = self.pos_three_stage[363][1] - self.pos_three_stage[362][1]
            epsilon_33 = ((thirtythree - 3.970001220709989)/3.970001220709989)*100
            #print(epsilon_33)
            
            r33 = 424.128464463/2711.890962292 *( 40.9786262984012 * epsilon_33 + 2711.890962292 )
            #print(r33) 
                    
            
             ######### thirty4 ###########
            thirtyfour = self.pos_three_stage[395][1] - self.pos_three_stage[394][1]
            epsilon_34 = ((thirtyfour - 3.968994140620012)/3.968994140620012)*100
            #print(epsilon_34)
            
            r34 = 424.02087442/2711.890962292 *( 40.9786262984012 * epsilon_34 + 2711.890962292 )
            #print(r34)
          
        
            ######### thirty_5 ###########
            thirtyfive = self.pos_three_stage[254][1] - self.pos_three_stage[273][1]
            #print(" thirty_5 is :",  thirty_5) 
            epsilon_35 = ((thirtyfive - 4.642997741699006)/4.642997741699006)*100
            #print(epsilon_35)
            
            r35 = 496.026925995/2711.890962292 *( 40.9786262984012 * epsilon_35 + 2711.890962292 )
            #print(r35)
    
            
            ######### thirty6 ###########
            thirtysix = self.pos_three_stage[170][1] - self.pos_three_stage[169][1]
            #print(" thirty_6 is :",  thirty_6) 
            epsilon_36 = ((thirtysix - 3.697998046874986)/3.697998046874986)*100
            #print(epsilon_36)
            
            r36 = 395.06946171/2711.890962292 *( 40.9786262984012 * epsilon_36 + 2711.890962292 )
            #print(r36)
          
           ######************ Kirchoff equations ****########
            

############# Zeros the matrices of A.I = C ##############
#### Where A are the coefficients, I is the current, C are the constants, and R are the resistances 
            A = np.zeros((42,42))
            I = np.zeros((42,1))
            C = np.zeros((42,1))
        ##### inpu current
            I[0] = 1
#print('The values are:', R[6], R[7], R[8])
#print(R)

############# Nodes first stage ##############
            A[0][0] = 1 ; A[0][6] = -1 ; A[0][12] = -1 ; A[0][11] = 1 ; 					
            A[1][1] = 1 ; A[1][7] = -1 ; A[1][13] = -1 ; A[1][6] = 1 ; 					
            A[2][2] = 1 ; A[2][8] = -1 ; A[2][14] = -1 ; A[2][7] = 1 ; 					
            A[3][3] = 1 ; A[3][9] = -1 ; A[3][15] = -1 ; A[3][8] = 1 ; 					
            A[4][4] = 1 ; A[4][10] = -1 ; A[4][16] = -1 ; A[4][9] = 1 ; 					
            A[5][5] = 1 ; A[5][11] = -1 ; A[5][17] = -1 ; A[5][10] = 1 ; 					


########### Nodes second stage ##############
            A[6][12] = 1 ; A[6][18] = -1 ; A[6][24] = -1 ; A[6][23] = 1 ; 					
            A[7][13] = 1 ; A[7][19] = -1 ; A[7][25] = -1 ; A[7][18] = 1 ; 					
            A[8][14] = 1 ; A[8][20] = -1 ; A[8][26] = -1 ; A[8][19] = 1 ; 					
            A[9][15] = 1 ; A[9][21] = -1 ; A[9][27] = -1 ; A[9][20] = 1 ; 					
            A[10][16] = 1 ; A[10][22] = -1 ; A[10][28] = -1 ; A[10][21] = 1 ; 					
            A[11][17] = 1 ; A[11][23] = -1 ; A[11][29] = -1 ; A[11][22] = 1 ; 					

########## Nodes third stage ##############
            A[12][24] = 1 ; A[12][30] = -1 ; A[12][36] = -1 ; A[12][35] = 1 ; 						
            A[13][25] = 1 ; A[13][31] = -1 ; A[13][37] = -1 ; A[13][30] = 1 ; 						
            A[14][26] = 1 ; A[14][32] = -1 ; A[14][38] = -1 ; A[14][31] = 1 ; 						
            A[15][27] = 1 ; A[15][33] = -1 ; A[15][39] = -1 ; A[15][32] = 1 ; 						
            A[16][28] = 1 ; A[16][34] = -1 ; A[16][40] = -1 ; A[16][33] = 1 ; 						
            A[17][29] = 1 ; A[17][35] = -1 ; A[17][41] = -1 ; A[17][34] = 1 ; 						



########## Loop First Stage ##############	
# defining the resistances#   the index +1  R[6] is R_7  ########
            A[18][6] = r13 ; A[18][13] = r7 ; A[18][18] = -r14 ; A[18][12] = -r1 ; 								
            A[19][7] = r19 ; A[19][14] = r36 ; A[19][19] = -r20 ; A[19][13] = -r7 ; 								
            A[20][8] = r22 ; A[20][15] = r6 ; A[20][20] = -r23 ; A[20][14] = -r36 ; 								
            A[21][9] = r25 ; A[21][16] = r12 ; A[21][21] = -r26 ; A[21][15] = -r6 ; 								
            A[22][10] = r28 ; A[22][17] = r31 ; A[22][22] = -r29 ; A[22][16] = -r12 ; 								
            A[23][11] = r16 ; A[23][12] = r1 ; A[23][23] = -r17 ; A[23][17] = -r31 ; 								

	
########## Loop Second Stage ##############		
            A[24][18] = r14 ; A[24][25] = r8 ; A[24][30] = -r15 ; A[24][24] = -r2 ; 												
            A[25][19] = r20 ; A[25][26] = r35 ; A[25][31] = -r21 ; A[25][25] = -r8 ; 							
            A[26][20] = r23 ; A[26][27] = r5 ; A[26][32] = -r24 ; A[26][26] = -r35 ; 							
            A[27][21] = r26 ; A[27][28] = r11 ; A[27][33] = -r27 ; A[27][27] = -r5 ; 							
            A[28][22] = r29 ; A[28][29] = r32 ; A[28][34] = -r30 ; A[28][28] = -r11 ; 							
            A[29][23] = r17 ; A[29][24] = r2 ; A[29][35] = -r18 ; A[29][29] = -r32; 							


########## Loop Third Stage ##############
            A[30][30] = r15 ; A[30][37] = r9 ; A[30][36] = -r3 ; 
            A[31][31] = r21 ; A[31][38] = r34  ; A[31][37] = -r9 ; 						
            A[32][32] = r24 ; A[32][39] = r4  ; A[32][38] = -r34 ; 						
            A[33][33] = r27 ; A[33][40] = r10  ; A[33][39] = -r4 ; 						
            A[34][34] = r30 ; A[34][41] = r33  ; A[34][40] = -r10 ; 						
            A[35][35] = r18 ; A[35][36] = r3  ; A[35][41] = -r33 ; 						


########### Boundary Conditions 1 and 4 main radial ############## 
            A[36][0] = 1 ; C[36][0] = 1 ;		
            A[37][1] = 1 ; C[37][0] = 0 ;		
            A[38][2] = 1 ; C[38][0] = 0 ;		
            A[39][3] = 1 ; C[39][0] = -1 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


## Solve the matrix equation
            I = solve(A, C)
            

###### Access the solution######
#####  I[7] is I_6 ########

            V = r13 * I[6] + r19* I[7] +r22 * I[8];
######## check current is the same  I25 = I24 acual order 

            v_c = r1 * I[12] +  r2 * I[24] +  r3 * I[36] + r4 * I[39] + r5 * I[27] +  r6 * I[15] 
            
            ##### parallel ####
            
            ##v_cp = ((I[6] + I[18] + I[30]) * ((r13 * r14 * r15)/(r13 + r14 + r15))) + ((I[7] + I[19] + I[31]) * ((r19 * r20 * r21)/(r19 + r20 + r21))) + ((I[8] + I[20] + I[32]) * ((r22 * r23 * r24)/(r22 + r23 + r24))) - ((I[11] + I[23] + I[35]) * ((r16 * r17 * r18)/(r16 + r17 + r18))) - ((I[10] + I[22] + I[34]) * ((r28 * r29 * r30)/(r28 + r29 + r30))) - ((I[9] + I[21] + I[33]) * ((r25 * r26 * r27)/(r25 + r26 + r27)))
            
            
            R_t14 = V;
            #print(R_t14)
            ##print(V, I[0], R_t)

########## Boundary Conditions 1 and 2 first spiral ############## 
            A[36][0] = 1 ; C[36][0] = 1 ;		
            A[37][1] = 1 ; C[37][0] = -1 ;		
            A[38][2] = 1 ; C[38][0] = 0 ;		
            A[39][3] = 1 ; C[39][0] = 0 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

##### Access the solution######
####  I[7] is I_6 ########

            V = r13 * I[6] ;
####### check current is the same 
            v_c = r16  * I[11] + r28 * I[10] + r25  * I[9] + r22 *  I[8] + r19 * I[7] 
            R_t12 = -1 * v_c 
            #print(R_t12)
             #### the perpendicular radial to force ############### check again the outer spirals and radials 
##current = I[10]
#print(current)


########## Boundary Conditions 2 and 5  ############## 
            A[36][0] = 1 ; C[36][0] = 0 ;		
            A[37][1] = 1 ; C[37][0] = 1 ;		
            A[38][2] = 1 ; C[38][0] = 0 ;		
            A[39][3] = 1 ; C[39][0] = 0 ;		
            A[40][4] = 1 ; C[40][0] = -1 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

            V = r13 * I[6] + r16 * I[11] + r28 * I[10]
            R_t25 = -1 * V
            #print(R_t25)
            
            
            ########## Boundary Conditions 3 and 6  ############## 
            A[36][0] = 1 ; C[36][0] = 0 ;		
            A[37][1] = 1 ; C[37][0] = 0 ;		
            A[38][2] = 1 ; C[38][0] = 1 ;		
            A[39][3] = 1 ; C[39][0] = 0 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = -1 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

            V = r28 * I[10] + r25 * I[9] + r22 * I[8]
            v_c = r16 * I[11] + r13 * I[6] + r19 * I[7]
            
            #R_t36 =  -1 * v_c
            R_t36 = V
            
            #print(R_t36)
            
        
            ########## Boundary Conditions 3 and 4  ############## 
            A[36][0] = 1 ; C[36][0] = 0 ;		
            A[37][1] = 1 ; C[37][0] = 0;		
            A[38][2] = 1 ; C[38][0] = 1 ;		
            A[39][3] = 1 ; C[39][0] = -1 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

            V = r22 * I[8]
            v_c =  r25 * I[9] + r28 * I[10] + r16 * I[11] +  r13 * I[6] +  r19 * I[7]
            
            #R_t34 =  -1 * v_c
            R_t34 =  V
            #print(R_t34)
            
            
                    
            ########## Boundary Conditions 2 and 3  ############## 
            A[36][0] = 1 ; C[36][0] = 0 ;		
            A[37][1] = 1 ; C[37][0] = 1 ;		
            A[38][2] = 1 ; C[38][0] = -1;		
            A[39][3] = 1 ; C[39][0] = 0 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

            V = r19 * I[7]
            v_c =  r13 * I[6] + r16 * I[11] + r28 * I[10] +  r25 * I[9] +  r22 * I[8]
            
            R_t23 =  -1 * v_c
            #R_t23 =  V
        
            #print(R_t23)
            
            
                        
                    
            ########## Boundary Conditions 1 and 6  ############## 
            A[36][0] = 1 ; C[36][0] = 1 ;		
            A[37][1] = 1 ; C[37][0] = 0 ;		
            A[38][2] = 1 ; C[38][0] = 0;		
            A[39][3] = 1 ; C[39][0] = 0 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = -1 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

            V = r16 * I[11]
            v_c =  r28 * I[10] + r25 * I[9] + r22 * I[8] +  r19 * I[7] +  r13 * I[6]
            
            R_t16 =   v_c
            #R_t16 =  V
        
            #print(R_t16)
            
            
                                
            ########## Boundary Conditions 5 and 6  ############## 
            A[36][0] = 1 ; C[36][0] = 0 ;		
            A[37][1] = 1 ; C[37][0] = 0 ;		
            A[38][2] = 1 ; C[38][0] = 0;		
            A[39][3] = 1 ; C[39][0] = 0 ;		
            A[40][4] = 1 ; C[40][0] = 1 ;		
            A[41][5] = 1 ; C[41][0] = -1 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

            V = r28 * I[10]
            v_c =  r16 * I[11] + r13 * I[6] + r19 * I[7] +  r22 * I[8] +  r25 * I[9]
            
            R_t56 =  -1 * v_c
            #R_t56 =  V
        
            #print(R_t56)
            
            
        ########## Boundary Conditions 4 and 5  ############## 
            A[36][0] = 1 ; C[36][0] = 0 ;		
            A[37][1] = 1 ; C[37][0] = 0 ;		
            A[38][2] = 1 ; C[38][0] = 0;		
            A[39][3] = 1 ; C[39][0] = 1 ;		
            A[40][4] = 1 ; C[40][0] = -1 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


# Solve the matrix equation
            I = solve(A, C);
            #I[0] # injec current

            V = r25 * I[9]
            v_c =  r22 * I[8] + r19 * I[7] + r13 * I[6] +  r16 * I[11] +  r28 * I[10]
            
            R_t54 =  -1 * v_c
            #R_t54 =  V
            #print(R_t54)
        
            
################# Check whether the solution is correct ################
            #print(np.allclose(np.dot(A,I),C))

   
      #######***********************  matrix strain   ************************#


            matrix_length = self.pos_matrix[1][1] - self.pos_matrix[6][1]
            #print(" matrix_length is :",   matrix_length)
            

 #######***********************  part1  segments   ************************#
            matrix_1 = self.pos_matrix[267][1] - self.pos_matrix[4][1]
            
            #print("matrix_1", matrix_1 )
            
            matrix_epsilon_1 = ((matrix_1 - 20.104896677387785)/20.104896677387785) * 100
            #print("matrix_epsilon_1", matrix_epsilon_1)
            

            #######***********************  part2  segments   ************************#
            matrix_2 = self.pos_matrix[215][1] - self.pos_matrix[267][1]
            
            #print("matrix_2", matrix_2 )
            
            matrix_epsilon_2 = ((matrix_2 - 20.441054740343915 )/20.441054740343915) * 100
            
            #print("matrix_epsilon_2", matrix_epsilon_2)
            
            #######***********************  part3  segments   ************************#
            matrix_3 = self.pos_matrix[147][1] - self.pos_matrix[215][1]
            
            #print("matrix_3", matrix_3 )
            
            matrix_epsilon_3 = ((matrix_3 - 20.56515969337761 )/20.56515969337761) * 100
            #print("matrix_epsilon_3", matrix_epsilon_3)
            
            #######***********************  part4  segments   ************************#
            matrix_4 = self.pos_matrix[221][1] - self.pos_matrix[147][1]
            
            #print("matrix_4", matrix_4)
            
            matrix_epsilon_4 = ((matrix_4 - 20.901317756333697 )/20.901317756333697) * 100
            #print("matrix_epsilon_4", matrix_epsilon_4)
            
            #######***********************  part5  segments   ************************#
            matrix_5 = self.pos_matrix[0][1] - self.pos_matrix[286][1]

            #print("matrix_5", matrix_5 )
            
            matrix_epsilon_5 = ((matrix_5 - 20.104896677387785 )/20.104896677387785) * 100
            #print("matrix_epsilon_5", matrix_epsilon_5)
            
            matrix_ep = (((matrix_1 - 20.104896677387785) + (matrix_2 - 20.441054740343915 ) + (matrix_3 - 20.56515969337761 ) + (matrix_4 - 20.901317756333697 ) + (matrix_5 - 20.104896677387785 ))/ (20.104896677387785 + 20.441054740343915 + 20.56515969337761 + 20.901317756333697 + 20.104896677387785)) * 100
            print(matrix_ep)
            

                

	
		
		


def createScene(rootNode):
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='SofaPython3')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')
    
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective') # Needed to use components [FixedConstraint]  
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [BarycentricMapping] 
    
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.MechanicalLoad') # Needed to use components [ConstantForceField]  
     
     
    rootNode.addObject('RequiredPlugin', pluginName=[
        "Sofa.Component.AnimationLoop",  # Needed to use components FreeMotionAnimationLoop
        "Sofa.Component.Constraint.Lagrangian.Correction",
        # Needed to use components LinearSolverConstraintCorrection
        "Sofa.Component.Constraint.Lagrangian.Solver",  # Needed to use components GenericConstraintSolver
        "Sofa.Component.Engine.Select",  # Needed to use components BoxROI
        "Sofa.Component.IO.Mesh",  # Needed to use components MeshSTLLoader, MeshVTKLoader
        "Sofa.Component.LinearSolver.Direct",  # Needed to use components SparseLDLSolver
        "Sofa.Component.Mass",  # Needed to use components UniformMass
        "Sofa.Component.ODESolver.Backward",  # Needed to use components EulerImplicitSolver
        "Sofa.Component.Setting",  # Needed to use components BackgroundSetting
        "Sofa.Component.SolidMechanics.FEM.Elastic",  # Needed to use components TetrahedronFEMForceField
        "Sofa.Component.SolidMechanics.Spring",  # Needed to use components RestShapeSpringsForceField
        "Sofa.Component.Topology.Container.Constant",  # Needed to use components MeshTopology
        "Sofa.Component.Visual",  # Needed to use components VisualStyle
        "Sofa.GL.Component.Rendering3D",  # Needed to use components OglModel
    ])
    rootNode.addObject('VisualStyle', displayFlags="showVisualModels hideBehaviorModels hideCollisionModels \
                        hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe")

    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
    rootNode.addObject('DefaultVisualManagerLoop')

    rootNode.addObject('GenericConstraintSolver', maxIterations=1000, tolerance=1e-3)

    #rootNode.addObject('BackgroundSetting', color=[0, 0.168627, 0.211765, 1])
    rootNode.addObject('BackgroundSetting', color=[0, 0, 0, 0])
    #rootNode.findData('gravity').value = [0, 0, -981.0]
    rootNode.findData('gravity').value = [0, 0, -9.81]


  
    rootNode.findData('dt').value = 0.01
     

     
    ##########################################
    # python script controller to control tensile force  
    ##########################################
  
    #rootNode.createObject('PythonScriptController', classname="controller", filename="force_controller.py")
    
    
  ##########################################
    # FEM Model                              #
    ##########################################
    matrix = rootNode.addChild('matrix')
    matrix.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.2, rayleighMass=0.2)
    matrix.addObject('SparseLDLSolver')
    
    matrix.addObject('MeshVTKLoader', name='loader', filename=path + 'rectangle.vtk', rotation=[0, 0, 0])
    matrix.addObject('MeshTopology', src='@loader')

    matrix.addObject('MechanicalObject', name='l_matrix', template='Vec3')
    matrix.addObject('UniformMass', totalMass=1)
    matrix.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.4,
                        youngModulus=0.6e6)
    
    matrix.addObject('FixedConstraint', indices="5 66 152 815 78 152 153 875 76 162 971 81 161 162 901 82 160 161 924 73 159 160 924 72 158 159 888 85 86 157 566 68 154 155 878 7 89 154 826")
    
    
    
    matrix.addObject('ConstantForceField',name='FF', indices= "  0 31 124 824 8 124 125 876 10 125 128 252 11 12 130 254 13 129 132 255 14 15 131 927 16 133 134 841 18 126 127 818 1 19 126 818", force =[0, 0, 0], showArrowSize = "0.00003")
    


    matrix.addObject('LinearSolverConstraintCorrection')
   
   
 

   
   ############################  strain control ########################
   
   #matrix.addObject('PositionConstraint', indices= "725 806 974 599 627 911 1125 505 519 832 1003 520 541 662 1039 466 581 644 734 892 951 960 1232 1449 1458 1462 224 296 462 541 154 156 393 540",valueType="displacement", value=50, useDirections=[0, 1, 0]) 


    
    
     # matrix visualization                     
    ##########################################
    # In Sofa, visualization is handled by adding a rendering model.
    #  Create an empty child node to store this rendering model.
    matrixVisu = matrix.addChild('visu0')

    # Add to this empty node a rendering model made of triangles and loaded from a stl file.
    matrixVisu.addObject('MeshSTLLoader', filename=path + "rectangle.stl", name="loader")
    matrixVisu.addObject('OglModel', src="@loader",  color=[0.9, 0.9, 0.9, 0.5]) 

    # Add a BarycentricMapping to deform the rendering model in a way that follow the ones of the parent mechanical model.
    matrixVisu.addObject('TriangleCollisionModel')
    matrixVisu.addObject('LineCollisionModel')
    matrixVisu.addObject('PointCollisionModel')

    matrixVisu.addObject('BarycentricMapping')
    

     ##########################################
    # three_stage                           #
    ##########################################
    #  This add a new node in the scene. This node is appended to the matrix's node.
    three_stage = matrix.addChild('three_stage')
    three_stage.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.2, rayleighMass=0.2)
    three_stage.addObject('SparseLDLSolver')
    #  This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a pneumatic actuation it is a set of positions describing the spider wall.
    three_stage.addObject('MeshVTKLoader', name='loader', filename=path + '3_stage_trim.vtk', rotation=[0, 0, 0])
    three_stage.addObject('MeshTopology', src='@loader', name='topo')
    three_stage.addObject('MechanicalObject', name='l_three_stage')
    three_stage.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.4,
                        youngModulus=9e6)
    three_stage.addObject('UniformMass', totalMass=0.003)
    #three_stage.addObject('LinearSolverConstraintCorrection')


    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    #  between the spider wall (surfacic mesh) and the matrix (volumetric mesh) so that movements of the spider's DoFs will be mapped
    #  to the matrix and vice-versa;
    three_stage.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
    three_stage.addObject(SpiderController(node=rootNode, pos_matrix = rootNode.matrix.l_matrix.position.value,  pos_three_stage = rootNode.matrix.three_stage.l_three_stage.position.value))

    ##########################################
    # three_stage Visualization                          
    ##########################################
    three_stageVisu = three_stage.addChild('visu1')
    three_stageVisu.addObject('MeshSTLLoader', filename=path + "3_stage_trim.stl", name="loader")
    three_stageVisu.addObject('OglModel', src="@loader", color=[0.1, 0.1, 0.1, 0.9])
        
    three_stageVisu.addObject('TriangleCollisionModel')
    three_stageVisu.addObject('LineCollisionModel')
    three_stageVisu.addObject('PointCollisionModel')

    three_stageVisu.addObject('BarycentricMapping')


    return rootNode

