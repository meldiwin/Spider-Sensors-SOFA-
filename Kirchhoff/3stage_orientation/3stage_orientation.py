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
            
           
           ################# Radials ######################
           
            ###### seventeen ####
            
            seventeen = self.pos_three_stage[2146][1] - self.pos_three_stage[2312][1]
            #print("seventeen is :", seventeen)
            
            epsilon_17 = ((seventeen - 3.7380464009115997 )/ 3.7380464009115997)*100
            #print(epsilon_12)
            
            
            r17 =  ((3.7380464009115997 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_17 + 2711.890962292 )
            
            #print(r17)
            
            ###### 29 ####
            
            twentynine = self.pos_three_stage[2379][1] - self.pos_three_stage[2570][1]
            #print("twentynine is :", twentynine)
            
            epsilon_29 = ((twentynine - 4.694805145265008)/4.694805145265008)*100
            r29 = ((4.694805145265008 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_29 + 2711.890962292 )
            
            #print(r29)
        
            
               ###### 41 ####
            
            fourtyone = self.pos_three_stage[2573][1] - self.pos_three_stage[2659][1]
            #print("fourtyone is :", fourtyone)
            
            epsilon_41 = ((fourtyone - 4.015838623050001)/4.015838623050001)*100
            
            #print(epsilon_41)
            
            r41 = ((4.015838623050001 * 3205)/30)/2711.890962292  *( 40.9786262984012 * epsilon_41 + 2711.890962292 )
            #print(r41)
            
        
            ###### 38 ####
            
            
            thirtyeight = self.pos_three_stage[2661][1] - self.pos_three_stage[2533][1]
            #print("thirtyeight is :", thirtyeight)
            
            epsilon_38 = ((thirtyeight - 4.013835906977505)/4.013835906977505)*100
            
            #print(epsilon_10)
            
            r38 = ((4.013835906977505 * 3205)/30)/2711.890962292  *( 40.9786262984012 * epsilon_41 + 2711.890962292 )
            #print(r38)
            
            
            ###### 26 ####
            
                    
            twentysix = self.pos_three_stage[2528][1] - self.pos_three_stage[2423][1]
            #print("twentysix is :", twentysix)
            
            
            epsilon_26 = ((twentysix  -  4.093830108642479)/ 4.093830108642479)*100
            #print(epsilon_11)
            
            r26 = ((4.013835906977505 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_26 + 2711.890962292 )
            
            #print(r26)
            
            
            
            ###### 14 ####
            
            fourteen = self.pos_three_stage[2393][1] - self.pos_three_stage[2233][1]
            #print("fourteen  is :", fourteen )
            
            epsilon_14 = ((fourteen - 4.459815979004006)/ 4.459815979004006)*100
            #print(epsilon_12)
            
            
            r14 = ((4.459815979004006 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_14 + 2711.890962292 )
            
            #print(r14)
                
            ######### 18 ###########
            eighteen = self.pos_three_stage[2293][0] - self.pos_three_stage[2346][0]
            #print("eighteen  is :", eighteen )
            
            epsilon_18 = ((eighteen - 4.391105651855504)/ 4.391105651855504)*100
            #print(epsilon_31)
            
            r18 =((4.391105651855504 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_18 + 2711.890962292 )
            #print(r18)
               
            ######### thirty ###########
            thirty  = self.pos_three_stage[2349][0] - self.pos_three_stage[2601][0]
            #print("thirty   is :", thirty  )
            
            epsilon_30 = ((thirty - 4.030788421631016 )/4.030788421631016 )*100
            #print(epsilon_30)
            
            r30 = ((4.030788421631016 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_30 + 2711.890962292 )
            #print(r30) 
            
              ######### 42 ###########
            fourtytwo = self.pos_three_stage[2604][0] - self.pos_three_stage[2650][0]
            
            #print("fourtytwo  is :", fourtytwo  )
            
            epsilon_42 = ((fourtytwo - 3.9511375427250144)/3.9511375427250144)*100
            #print(epsilon_33)
            
            r42 = (( 3.9511375427250144 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_42 + 2711.890962292 )
            #print(r42) 
            
               ######### 39 ###########
            thirtynine = self.pos_three_stage[2643][0] - self.pos_three_stage[2550][0]
            
            #print("thirtynine  is :", thirtynine )
            
            epsilon_39 = ((thirtynine - 3.9511451721190056)/3.9511451721190056)*100
            #print(epsilon_34)
            
            r39 = ((3.9511451721190056 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_39 + 2711.890962292 )
            #print(r39)
          
                    
            ######### 27 ###########
            twentyseven = self.pos_three_stage[2548][0] - self.pos_three_stage[2430][0]
            #print(" twentyseven is :",  twentyseven) 
            
            epsilon_27 = ((twentyseven - 4.62103652954049)/4.62103652954049)*100
            #print(epsilon_35)
            
            r27 = ((4.62103652954049 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_27 + 2711.890962292 )
            #print(r27)
    
              ######### 15 ###########
            fifteen = self.pos_three_stage[2424][0] - self.pos_three_stage[2206][0]
            
            #print("fifteen is :",  fifteen) 
            
            epsilon_15 = ((fifteen - 3.681396484374993)/3.681396484374993)*100
            #print(epsilon_36)
            
            r15 = ((3.681396484374993 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_15 + 2711.890962292 )
            #print(r15)
            
            
            
            ###### thirteen ####
            
            thirteen = self.pos_three_stage[2259][0] - self.pos_three_stage[2385][0]
            #print("thirteen is :", thirteen)
            
            epsilon_13 = ((thirteen - 3.7158012390135085)/3.7158012390135085)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r13 = ((3.7158012390135085 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_13 + 2711.890962292 )
            #print(r1)
            
                 
             ###### ten ####
            
            twentyfive = self.pos_three_stage[2388][0] - self.pos_three_stage[2520][0]
            #print("twentyfive is :", twentyfive)
            
            epsilon_25 = ((twentyfive - 4.663585662841996)/4.663585662841996)*100
            
            r25  = ((4.663585662841996 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_25 + 2711.890962292 )
            #print(r25)
            
            
                 ###### thirtyseven ####
            
            thirtyseven = self.pos_three_stage[2524][0] - self.pos_three_stage[2649][0]
            #print("thirtyseven is :", thirtyseven)
            
            epsilon_37 = ((thirtyseven - 3.987529754638487)/3.987529754638487)*100
            
            #print(epsilon_3)
            
            r37 =  ((3.987529754638487* 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_37 + 2711.890962292 )
            #print(r37)
            
            
             ###### fourty ####
            
            fourty = self.pos_three_stage[2642][0] - self.pos_three_stage[2593][0]
            #print("fourty is :", fourty)
            
            epsilon_40 = ((fourty - 3.9875373840330113)/3.9875373840330113)*100
            #print(epsilon_4)
            
            r40 = ((3.9875373840330113* 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_40 + 2711.890962292 )
            #print(r40)
            
            
                                                 
             ###### 28 ####
            
            twentyeight = self.pos_three_stage[2590][0] - self.pos_three_stage[2306][0]
            #print("twentyeight is :", twentyeight)
            
            epsilon_28 = ((twentyeight - 4.067878723144503)/4.067878723144503)*100
            #print(epsilon_5)
            
            r28 = ((4.067878723144503 * 3205)/30)/2711.890962292  *( 40.9786262984012 * epsilon_28 + 2711.890962292 )
            #print(r28)
                          
                          
               ###### sixteen ####
            
            sixteen = self.pos_three_stage[2303][0] - self.pos_three_stage[2172][0]
            #print("sixteen is :", sixteen)
            
            epsilon_16 = ((sixteen - 4.431533813476506)/4.431533813476506)*100
            #print(epsilon_6)
            
            r16 = ((4.431533813476506 * 3205)/30)/2711.890962292 *( 40.9786262984012 * epsilon_16 + 2711.890962292 )
            
            #print(r16) 
                   
                   
                   
        ############### Spirals 1 ##################################
        
         ######### eleven ###########
            eleven = self.pos_three_stage[73][0] - self.pos_three_stage[7][0]
            #print("eleven is :", eleven)
            
            ########### segment1 ################### 
            
            eleven_1 = self.pos_three_stage[73][0] - self.pos_three_stage[2296][0]
            #print("eleven_1 is :", eleven_1)
            
            epsilon_11_1 = ((eleven_1 - 2.9390258789060084)/2.9390258789060084)*100
            r11_1 = ((R_spiral_1* 2.9390258789060084)/14.401115417479993) /2711.890962292 *( 40.9786262984012 * epsilon_11_1 + 2711.890962292 )
            #print (r11_1)
                        
            ########### segment2 ################### 
            
            eleven_2 = self.pos_three_stage[2296][0] - self.pos_three_stage[5935][0] #1333 #
            #print("eleven_2 is :", eleven_2)
            
            epsilon_11_2 = ((eleven_2 - 2.867624282836772)/2.867624282836772)*100
            r11_2 = ((R_spiral_1* 2.867624282836772)/14.401115417479993) /2711.890962292 *( 40.9786262984012 * epsilon_11_2 + 2711.890962292 )
            #print (r11_2)
            
            ########### segment3 ################### 
            
            eleven_3 = self.pos_three_stage[5935][0] - self.pos_three_stage[4525][0]  
            #print("eleven_3 is :", eleven_3)
            
            epsilon_11_3 = ((eleven_3 - 2.8876609802255047)/ 2.8876609802255047)*100
            r11_3 =  ((R_spiral_1* 2.8876609802255047)/14.401115417479993)/2711.890962292 *( 40.9786262984012 * eleven_3 + 2711.890962292 )
            #print (r11_3)
            ########### segment 4 ################### 
            
            eleven_4 = self.pos_three_stage[4525][0] - self.pos_three_stage[5467][0]
            #print("eleven_4 is :", eleven_4)
            
            epsilon_11_4 = ((eleven_4 - 2.9143238067610184)/ 2.9143238067610184)*100
            r11_4 = ((R_spiral_1 * 2.9143238067610184 )/14.401115417479993)/2711.890962292 *( 40.9786262984012 * epsilon_11_4 + 2711.890962292 )
            #print (r11_4)
             ########### segment 5 ################### 
            
            eleven_5 = self.pos_three_stage[5467][0] - self.pos_three_stage[7][0]
            #print("eleven_5 is :", eleven_5)
            
            epsilon_11_5 = ((eleven_5 - 2.7924804687506892)/ 2.7924804687506892)*100
            r11_5 = ((R_spiral_1 * 2.7924804687506892)/14.401115417479993)/2711.890962292 *( 40.9786262984012 * epsilon_11_5 + 2711.890962292 )
            #print (r11_5)
            
            r11 = r11_1 + r11_2 + r11_3 + r11_4 + r11_5
            #print (r11)
            
            
             ######### 12 ###########
            twelve = self.pos_three_stage[72][1] - self.pos_three_stage[60][1]
            #print("twelve is :", twelve)
            
            ########### segment 1 #####################
            twelve_1 = self.pos_three_stage[72][1] - self.pos_three_stage[873][1]
            #print("twelve_1_1 is :", twelve_1)
            
            epsilon_12_1 = ((twelve_1 - 2.9884757995599927 )/2.9884757995599927)*100
            r12_1 =((R_spiral_1 * 2.9884757995599927)/14.495407104489004)/2711.890962292 *( 40.9786262984012 * epsilon_12_1 + 2711.890962292 )

            
            ########### segment 2 #####################
            twelve_2 = self.pos_three_stage[873][1] - self.pos_three_stage[5417][1]
            #print("twelve_2 is :", twelve_2)
            
            epsilon_12_2 = ((twelve_2 - 2.7815170288074995 )/ 2.7815170288074995)*100
            r12_2 = ((R_spiral_1 * 2.7815170288074995)/14.495407104489004)/2711.890962292 *( 40.9786262984012 * epsilon_12_2 + 2711.890962292 )


            ########### segment 3 #####################
            twelve_3 = self.pos_three_stage[5417][1] - self.pos_three_stage[65][1]
            #print("twelve_3  is :", twelve_3 )
            
            epsilon_12_3 = ((twelve_3 - 2.7132759094224923 )/2.7132759094224923)*100
            r12_3 = ((R_spiral_1 * 2.7132759094224923)/14.495407104489004)/2711.890962292 *( 40.9786262984012 * epsilon_12_3 + 2711.890962292 )

            
            
            ########### segment 4 #####################
            twelve_4 = self.pos_three_stage[65][1] - self.pos_three_stage[1006][1]
            #print("twelve_4 is :", twelve_4)
        
            epsilon_12_4 = ((twelve_4 - 2.862171173095973 )/ 2.862171173095973)*100
            r12_4 = ((R_spiral_1 * 2.862171173095973)/14.495407104489004)/2711.890962292 *( 40.9786262984012 * epsilon_12_4 + 2711.890962292 )

            
            ########### segment 4 #####################
            twelve_5 = self.pos_three_stage[1006][1] - self.pos_three_stage[60][1]
            #print("twelve_5 is :", twelve_5)
            
                        
            epsilon_12_5 = ((twelve_5 - 3.1499671936030182)/ 3.1499671936030182)*100
            r12_5 =((R_spiral_1 * 3.1499671936030182)/14.495407104489004)/2711.890962292 *( 40.9786262984012 * epsilon_12_5 + 2711.890962292 )
            
            r12 =  r12_1 +  r12_2 +  r12_3 +  r12_4 +  r12_5
            #print(r12)
            
            
             
               ######### seven ###########
            seven = self.pos_three_stage[59][0] - self.pos_three_stage[47][0]
            #print("seven is :", seven)
            
            ########### segment 1 #####################
            
            seven_1 = self.pos_three_stage[59][0] - self.pos_three_stage[2251][0]
            
            #print("seven_1 is :", seven_1)
            
            epsilon_7_1 = ((seven_1 - 2.9297447204589915 )/ 2.9297447204589915)*100
    
            r7_1 =((R_spiral_1 * 2.9297447204589915)/14.26972961425799)/2711.890962292 *( 40.9786262984012 * epsilon_7_1 + 2711.890962292 )
            
            
            ########### segment 2 #####################
            seven_2 = self.pos_three_stage[2251][0] - self.pos_three_stage[136][0]
            
            #print("seven_2 is :", seven_2)
            
            epsilon_7_2 = ((seven_2 - 2.7320976257320027)/ 2.7320976257320027)*100
            r7_2 = ((R_spiral_1 * 2.7320976257320027)/14.26972961425799)/2711.890962292 *( 40.9786262984012 * epsilon_7_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            
            seven_3 = self.pos_three_stage[136][0] - self.pos_three_stage[2243][0]
            
            #print("seven_3 is :", seven_3)
            
            epsilon_7_3 = ((seven_3 - 2.8218154907225)/ 2.8218154907225)*100
            r7_3 = ((R_spiral_1 * 2.8218154907225)/14.26972961425799)/2711.890962292 *( 40.9786262984012 * epsilon_7_3 + 2711.890962292 )
            
            
            ########### segment 4 #####################
            
            seven_4 = self.pos_three_stage[2243][0] - self.pos_three_stage[141][0]
            
            #print("seven_4 is :", seven_4)
            
            epsilon_7_4 = ((seven_4 - 2.854316711426513 )/ 2.854316711426513)*100
            
            r7_4 = ((R_spiral_1 * 2.854316711426513)/14.26972961425799)/2711.890962292 *( 40.9786262984012 * epsilon_7_4 + 2711.890962292 )
            
            
            ########### segment 5 #####################
            
            seven_5 = self.pos_three_stage[141][0] - self.pos_three_stage[47][0]
            
            #print("seven_5 is :", seven_5)
            
            epsilon_7_5 = ((seven_5 - 2.931755065917983)/ 2.931755065917983)*100
            r7_5 = ((R_spiral_1 * 2.931755065917983)/14.26972961425799)/2711.890962292 *( 40.9786262984012 * epsilon_7_5 + 2711.890962292 )
            
            r7 = r7_1 + r7_2 + r7_3 + r7_4 + r7_5 
            #print(r7)
            
            
            ######### eight ###########
            eight = self.pos_three_stage[46][0] - self.pos_three_stage[34][0]
            #print("eight is :", eight)
            
            ########### segment 1 #####################
            eight_1 = self.pos_three_stage[46][0] - self.pos_three_stage[3863][0]
            #print("eight_1 is :", eight_1) 
                                    
            epsilon_8_1 = (( eight_1  - 2.8462944030763495)/ 2.8462944030763495)*100
            r8_1 = ((R_spiral_1 * 2.8462944030763495 )/14.401100158691996)/2711.890962292 *( 40.9786262984012 * epsilon_8_1 + 2711.890962292 )
         
            ########### segment 2 #####################
            eight_2 = self.pos_three_stage[3863][0] - self.pos_three_stage[3859][0]
            #print("eight_2 is :", eight_2) 
                                    
            epsilon_8_2 = (( eight_2  - 2.860502243041644)/ 2.860502243041644)*100
            r8_2 =  ((R_spiral_1 * 2.860502243041644 )/14.401100158691996)/2711.890962292 *( 40.9786262984012 * epsilon_8_2 + 2711.890962292 )
        
            ########### segment 3 #####################
            eight_3 = self.pos_three_stage[3859][0] - self.pos_three_stage[5936][0]
            #print("eight_3 is :", eight_3) 
            
            epsilon_8_3 = (( eight_3  -  2.8876552581790023)/ 2.8876552581790023)*100
            r8_3 = ((R_spiral_1 * 2.8876552581790023 )/14.401100158691996)/2711.890962292 *( 40.9786262984012 * epsilon_8_3 + 2711.890962292 )
         
            ########### segment 4 #####################
            eight_4 = self.pos_three_stage[5936][0] - self.pos_three_stage[2210][0]
            #print("eight_4 is :", eight_4) 
            
            epsilon_8_4 = (( eight_4  - 2.867626190185497)/  2.867626190185497)*100
            r8_4 = ((R_spiral_1 * 2.867626190185497 )/14.401100158691996)/2711.890962292 *( 40.9786262984012 * epsilon_8_4 + 2711.890962292 )
         
            ########### segment 4 #####################
            eight_5 = self.pos_three_stage[2210][0] - self.pos_three_stage[34][0]
            #print("eight_5 is :", eight_5) 
            
            epsilon_8_5 = (( eight_5  - 2.939022064209503)/ 2.939022064209503)*100
            r8_5 = ((R_spiral_1 * 2.939022064209503 )/14.401100158691996)/2711.890962292 *( 40.9786262984012 * epsilon_8_5 + 2711.890962292 )
         
            r8 = r8_1 + r8_2 + r8_3 + r8_4 + r8_5
            #print(r8)
            
            ######### nine ###########
            nine = self.pos_three_stage[21][1] - self.pos_three_stage[33][1]
            #print("nine is :", nine)
           
            ########### segment1 #################### 
            nine_1 = self.pos_three_stage[21][1] - self.pos_three_stage[827][1]
            #print("nine_1  is :", nine_1 ) 
            
            epsilon_9_1 = (( nine_1   - 2.9742851257298923)/ 2.9742851257298923)*100
            r9_1 = ((R_spiral_1 *  2.9742851257298923)/14.495407104483007)/2711.890962292 *( 40.9786262984012 *  epsilon_9_1 + 2711.890962292 )
            
            ########### segment2 #################### 
            nine_2 = self.pos_three_stage[827][1] - self.pos_three_stage[3385][1]
            #print("nine_2  is :", nine_2 ) 
            
            epsilon_9_2 = (( nine_2   - 2.733352661127711)/ 2.733352661127711)*100
            r9_2 = ((R_spiral_1 *  2.733352661127711)/14.495407104483007)/2711.890962292 *( 40.9786262984012 *  epsilon_9_2 + 2711.890962292 )
            
                      
            ########### segment3 #################### 
            nine_3 = self.pos_three_stage[3385][1] - self.pos_three_stage[3367][1]
            #print("nine_3  is :", nine_3 ) 
            
            epsilon_9_3 = (( nine_3   - 3.060123443602137)/ 3.060123443602137)*100
            r9_3 = ((R_spiral_1 *  3.060123443602137)/14.495407104483007)/2711.890962292 *( 40.9786262984012 *  epsilon_9_3 + 2711.890962292 )
            
            ########### segment4 #################### 
            nine_4 = self.pos_three_stage[3367][1] - self.pos_three_stage[834][1]
            #print("nine_4  is :", nine_4) 
            
            epsilon_9_4 = (( nine_4   - 2.73917388915973)/2.73917388915973)*100
            r9_4 = ((R_spiral_1 *  2.73917388915973 )/14.495407104483007)/2711.890962292 *( 40.9786262984012 *  epsilon_9_4 + 2711.890962292 )  
            
                        
            ########### segment5 #################### 
            nine_5 = self.pos_three_stage[834][1] - self.pos_three_stage[33][1]
            #print("nine_5  is :", nine_5) 
            
            epsilon_9_5 = (( nine_5   - 2.988471984863537)/ 2.988471984863537)*100
            r9_5 = ((R_spiral_1 *  2.988471984863537 )/14.495407104483007)/2711.890962292 *( 40.9786262984012 *  epsilon_9_5 + 2711.890962292 )                        
            
            r9 = r9_1 + r9_2 + r9_3 + r9_4 + r9_5
            #print(r9)
            
            
             ######### ten ###########
            ten = self.pos_three_stage[8][0] - self.pos_three_stage[20][0]
            #print("ten is :", ten)
            
            ########### segment1 ################### 
            ten_1 = self.pos_three_stage[8][0] - self.pos_three_stage[3414][0]
            #print("ten_1 is :", ten_1) 
            
            epsilon_10_1 = (( ten_1  - 2.804950714111861)/ 2.804950714111861)*100
            r10_1 = ((R_spiral_1 * 2.804950714111861)/14.269714355468999)/2711.890962292 *( 40.9786262984012 * epsilon_10_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            ten_2 = self.pos_three_stage[3414][0] - self.pos_three_stage[3430][0]
            #print("ten_2 is :", ten_2) 
            
            epsilon_10_2 = (( ten_2  - 2.828275680542049)/ 2.828275680542049)*100
            r10_2 = ((R_spiral_1 * 2.828275680542049)/14.269714355468999)/2711.890962292 *( 40.9786262984012 * epsilon_10_2 + 2711.890962292 )
            
            ########### segment3 ################### 
            ten_3 = self.pos_three_stage[3430][0] - self.pos_three_stage[84][0]
            #print("ten_3 is :", ten_3) 
            
            epsilon_10_3 = (( ten_3  - 2.9746456146230855)/ 2.9746456146230855)*100
            r10_3 = ((R_spiral_1 * 2.9746456146230855)/14.269714355468999)/2711.890962292 *( 40.9786262984012 * epsilon_10_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            ten_4 = self.pos_three_stage[84][0] - self.pos_three_stage[460][0]
            #print("ten_4 is :", ten_4)
            
            epsilon_10_4 = (( ten_4  - 2.9821395874030046)/ 2.9821395874030046)*100
            r10_4 = ((R_spiral_1 * 2.9821395874030046)/14.269714355468999)/2711.890962292 *( 40.9786262984012 * epsilon_10_4 + 2711.890962292 )
            
                        
            ########### segment5 ################### 
            ten_5 = self.pos_three_stage[460][0] - self.pos_three_stage[20][0]
            #print("ten_5 is :", ten_5) 
            
            epsilon_10_5 = (( ten_5  - 2.6797027587889986)/ 2.6797027587889986)*100
            r10_5 = ((R_spiral_1 * 2.6797027587889986)/14.269714355468999)/2711.890962292 *( 40.9786262984012 * epsilon_10_5 + 2711.890962292 )
            
            r10 = r10_1 + r10_2 + r10_3 + r10_4 + r10_5
            #print(r10)
        
            
            
            
    ############### Spirals 2 ##################################
            
            
             ######### twentyone ###########
            twentythree = self.pos_three_stage[110][0] - self.pos_three_stage[122][0]
            #print(twentythree)
            
                        
            ######### segment 1 ###########
            twentythree_1 = self.pos_three_stage[110][0] - self.pos_three_stage[510][0]
            #print("twentythree_1 is :", twentythree_1)
            
            epsilon_23_1 = ((twentythree_1 - 1.8007431030269956)/ 1.8007431030269956)*100
            r23_1 = ((R_spiral_2 * 1.8007431030269956)/9.364852905274013) /2711.890962292 *( 40.9786262984012 * epsilon_23_1 + 2711.890962292 )
            
                             
            ######### segment 2 ###########
            twentythree_2 = self.pos_three_stage[510][0] - self.pos_three_stage[4568][0]
            #print("twentythree_2 is :", twentythree_2)
            
            epsilon_23_2 = ((twentythree_2 - 1.9358749389654548)/ 1.9358749389654548)*100
            r23_2 = ((R_spiral_2 * 1.9358749389654548)/9.364852905274013) /2711.890962292 *( 40.9786262984012 * epsilon_23_2 + 2711.890962292 )
            
            ######### segment 3 ###########
            twentythree_3 = self.pos_three_stage[4568][0] - self.pos_three_stage[4561][0]
            #print("twentythree_3 is :", twentythree_3)
            
            epsilon_23_3 = ((twentythree_3 - 2.0144252777116662)/2.0144252777116662)*100
            r23_3 = ((R_spiral_2 * 2.0144252777116662)/9.364852905274013) /2711.890962292 *( 40.9786262984012 * epsilon_23_3 + 2711.890962292 )
            
            ######### segment 4 ###########
            twentythree_4 = self.pos_three_stage[4561][0] - self.pos_three_stage[2942][0]
            #print("twentythree_4 is :", twentythree_4 )
            
            epsilon_23_4 = ((twentythree_4 - 1.7116985321023819)/ 1.7116985321023819)*100
            r23_4 = ((R_spiral_2 * 1.7116985321023819)/9.364852905274013) /2711.890962292 *( 40.9786262984012 * epsilon_23_4 + 2711.890962292 )
            
            ######### segment 5 ###########
            twentythree_5 = self.pos_three_stage[2942][0] - self.pos_three_stage[122][0]
            #print("twentythree_5 is :", twentythree_5)
            
            epsilon_23_5 = ((twentythree_5 - 1.9021110534675145)/1.9021110534675145)*100
            r23_5 =  ((R_spiral_2 * 1.9021110534675145)/9.364852905274013) /2711.890962292 *( 40.9786262984012 * epsilon_23_5 + 2711.890962292 )
            
            r23 = r23_1 + r23_2 + r23_3 + r23_4 + r23_5
            #print(r23)
            
              ######### twentyfour ###########
            twentyfour = self.pos_three_stage[182][1] - self.pos_three_stage[196][1]
            #print("twentyfour is :", twentyfour)
            
            ########### segment 1 #####################
            twentyfour_1 = self.pos_three_stage[182][1] - self.pos_three_stage[5187][1]
            #print("twentyfour_1 is :", twentyfour_1)
                 
            epsilon_24_1 = ((twentyfour_1 - 1.9404239654599849)/ 1.9404239654599849)*100
            r24_1 = ((R_spiral_2 * 1.9404239654599849)/9.77413940429598)/2711.890962292 *( 40.9786262984012 * epsilon_24_1 + 2711.890962292 )
            
            
            ########### segment 2 #####################
            twentyfour_2 = self.pos_three_stage[5187][1] - self.pos_three_stage[2475][1]
            #print("twentyfour_2 is :", twentyfour_2)
            
            epsilon_24_2 = ((twentyfour_2 -   1.9970447518511918 )/  1.9970447518511918)*100
            r24_2 = ((R_spiral_2 * 1.9970447518511918 )/9.77413940429598)/2711.890962292 *( 40.9786262984012 * epsilon_24_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            twentyfour_3 = self.pos_three_stage[2475][1] - self.pos_three_stage[4889][1]
            #print("twentyfour_3 is :", twentyfour_3)
            
                 
            epsilon_24_3 = ((twentyfour_3 -  2.038714220310297)/2.038714220310297)*100
            r24_3 = ((R_spiral_2 * 2.038714220310297)/9.77413940429598)/2711.890962292 *( 40.9786262984012 * epsilon_24_3 + 2711.890962292 )
            
            
            ########### segment 4 #####################
            twentyfour_4 = self.pos_three_stage[4889][1] - self.pos_three_stage[1012][1]
            #print("twentyfour_4 is :", twentyfour_4)
            
                 
            epsilon_24_4 = ((twentyfour_4 - 1.829553604125536)/ 1.829553604125536)*100
            r24_4 = ((R_spiral_2 * 1.829553604125536)/9.77413940429598)/2711.890962292 *( 40.9786262984012 * epsilon_24_4 + 2711.890962292 )
            
            
            ########### segment 5 #####################
            twentyfour_5 = self.pos_three_stage[1012][1] - self.pos_three_stage[196][1]
            #print("twentyfour_5 is :", twentyfour_5)
            
            epsilon_24_5 = ((twentyfour_5 -  1.9684028625489702)/ 1.9684028625489702)*100
            r24_5 = ((R_spiral_2 * 1.9684028625489702)/9.77413940429598)/2711.890962292 *( 40.9786262984012 * epsilon_24_5 + 2711.890962292 )
            
            r24 = r24_1 + r24_2 + r24_3 + r24_4 + r24_5
            #print(r24)
                
                ######### nineteen ###########
            nineteen = self.pos_three_stage[131][0] - self.pos_three_stage[145][0]
            #print(nineteen)
            
            ########### segment 1 #####################
            nineteen_1 = self.pos_three_stage[131][0] - self.pos_three_stage[155][0]
            #print(nineteen_1)
            
            epsilon_19_1 = ((nineteen_1 - 1.853950500489013)/1.853950500489013)*100            
            r19_1 = ((R_spiral_2 * 1.9684028625489702)/9.364852905274013)/2711.890962292 *( 40.9786262984012 * epsilon_19_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            nineteen_2 = self.pos_three_stage[155][0] - self.pos_three_stage[1057][0]
            
            #print("nineteen_2 is :", nineteen_2)
            
            epsilon_19_2 = ((nineteen_2 - 1.8555335998534304)/ 1.8555335998534304 )*100            
            r19_2 = ((R_spiral_2 * 1.8555335998534304)/9.364852905274013)/2711.890962292 *( 40.9786262984012 * epsilon_9_2 + 2711.890962292 )
            
            ########### segment 3 #####################
            nineteen_3 = self.pos_three_stage[1057][0] - self.pos_three_stage[2413][0]
            #print("nineteen_3 is :", nineteen_3)
            
            epsilon_19_3 = ((nineteen_3 - 1.847714147607789 )/ 1.847714147607789)*100            
            r19_3 = ((R_spiral_2 * 1.847714147607789)/9.364852905274013)/2711.890962292 *( 40.9786262984012 * epsilon_19_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            nineteen_4 = self.pos_three_stage[2413][0] - self.pos_three_stage[958][0]
            #print("nineteen_4 is :", nineteen_4)
            
            epsilon_19_4 = ((nineteen_4 - 1.8710823059079615 )/1.8710823059079615)*100            
            r19_4 = ((R_spiral_2 * 1.8710823059079615)/9.364852905274013)/2711.890962292 *( 40.9786262984012 * epsilon_19_4 + 2711.890962292 )
            
                        
            ########### segment 5 #####################
            nineteen_5 = self.pos_three_stage[150][0] - self.pos_three_stage[6404][0]
            #print("nineteen_5 is :", nineteen_5)
            
            epsilon_19_5 = ((nineteen_5 - 1.8572864532469424)/ 1.8572864532469424)*100            
            r19_5 = ((R_spiral_2 * 1.8572864532469424)/9.364852905274013)/2711.890962292 *( 40.9786262984012 * epsilon_19_5 + 2711.890962292 )
            
            r19 = r19_1 + r19_2 + r19_3 + r19_4 + r19_5
            #print(r19)
            
             
            ######### twenty ###########
            twenty = self.pos_three_stage[156][0] - self.pos_three_stage[170][0]
            #print("twenty is :", twenty)
            
            ########### segment 1 #####################
            twenty_1 = self.pos_three_stage[156][0] - self.pos_three_stage[264][0]
            #print("twenty_1 is :", twenty_1) 
            
            epsilon_20_1 = (( twenty_1  - 2.0548934936519956)/2.0548934936519956)*100
            
            r20_1 = ((R_spiral_2 * 2.0548934936519956)/10.068008422851996)/2711.890962292 *( 40.9786262984012 * epsilon_20_1 + 2711.890962292 )
    
            ########### segment 2 #####################
            twenty_2 = self.pos_three_stage[264][0] - self.pos_three_stage[2117][0]
            #print("twenty_2 is :", twenty_2) 
            
            epsilon_20_2 = (( twenty_2  - 1.9984754537255327)/1.9984754537255327)*100
            
            r20_2 = ((R_spiral_2 * 1.9984754537255327)/10.068008422851996)/2711.890962292 *( 40.9786262984012 * epsilon_20_2 + 2711.890962292 )
    
            ########### segment 3 #####################
            twenty_3 = self.pos_three_stage[2117][0] - self.pos_three_stage[6097][0]
            #print("twenty_3 is :", twenty_3) 
        
            epsilon_20_3 = (( twenty_3  - 2.098938562998555 )/ 2.098938562998555)*100
            
            r20_3 = ((R_spiral_2 * 2.098938562998555)/10.068008422851996)/2711.890962292 *( 40.9786262984012 * epsilon_20_3 + 2711.890962292 )
    
        
            ########### segment 4 #####################
            twenty_4 = self.pos_three_stage[6097][0] - self.pos_three_stage[4140][0]
            #print("twenty_4 is :", twenty_4) 
            
            epsilon_20_4 = (( twenty_4  - 1.8265438079831 )/ 1.8265438079831)*100
            
            r20_4 = ((R_spiral_2 * 1.8265438079831 )/10.068008422851996)/2711.890962292 *( 40.9786262984012 * epsilon_20_4 + 2711.890962292 )
    
            
            ########### segment 5 #####################
            twenty_5 = self.pos_three_stage[4140][0] - self.pos_three_stage[170][0]
            #print("twenty_5 is :", twenty_5) 
                
            epsilon_20_5 = (( twenty_5  - 2.0891571044928128)/ 2.0891571044928128)*100
            
            r20_5 = ((R_spiral_2 * 2.0891571044928128 )/10.068008422851996)/2711.890962292 *( 40.9786262984012 * epsilon_20_5 + 2711.890962292 )
    
            
            r20 = r20_1 + r20_2 + r20_3 + r20_4 + r20_5
            #print(r20)
            
            ######### twentyone ###########
            twentyone = self.pos_three_stage[222][1] - self.pos_three_stage[208][1]
            #print("twentyone is :", twentyone)
            
            ########### segment1 #################### 
            twentyone_1 = self.pos_three_stage[222][1] - self.pos_three_stage[3512][1]
            #print("twentyone_1  is :", twentyone_1) 
            
            epsilon_21_1 = (( twentyone_1   - 1.8305606842072848)/1.8305606842072848)*100
            r21_1 = ((R_spiral_2 * 1.8305606842072848)/9.787063598627995)/2711.890962292 *( 40.9786262984012 *  epsilon_21_1 + 2711.890962292 )   
            
            ########### segment2 ################### 
            twentyone_2 = self.pos_three_stage[3512][1] - self.pos_three_stage[3503][1]
            #print("twentyone_2  is :", twentyone_2) 
            
            epsilon_21_2 = (( twentyone_2   - 2.0674877166702004)/ 2.0674877166702004)*100
            r21_2 = ((R_spiral_2 * 2.0674877166702004)/9.787063598627995)/2711.890962292 *( 40.9786262984012 *  epsilon_21_2 + 2711.890962292 )   
            
            ########### segment3 ################### 
            twentyone_3 = self.pos_three_stage[3503][1] - self.pos_three_stage[3487][1]
            #print("twentyone_3  is :", twentyone_3) 
            
            epsilon_21_3 = (( twentyone_3   - 2.0707855224577116)/ 2.0707855224577116)*100
            r21_3 = ((R_spiral_2 * 2.0707855224577116)/9.787063598627995)/2711.890962292 *( 40.9786262984012 *  epsilon_21_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentyone_4 = self.pos_three_stage[3487][1] - self.pos_three_stage[1047][1]
            #print("twentyone_4  is :", twentyone_4) 
            
            epsilon_21_4 = (( twentyone_4   - 1.8367958068842398)/ 1.8367958068842398)*100
            r21_4 = ((R_spiral_2 * 1.8367958068842398)/9.787063598627995)/2711.890962292 *( 40.9786262984012 *  epsilon_21_4 + 2711.890962292 )
            
            
            ########### segment5 ################### 
            twentyone_5 = self.pos_three_stage[1047][1] - self.pos_three_stage[208][1]
            #print("twentyone_5  is :", twentyone_5) 
            
            epsilon_21_5 = (( twentyone_5   - 1.9814338684085584)/ 1.9814338684085584)*100
            r21_5 = ((R_spiral_2 * 1.9814338684085584)/9.787063598627995)/2711.890962292 *( 40.9786262984012 *  epsilon_21_5 + 2711.890962292 )
            
            r21 = r21_1 + r21_2 + r21_3 + r21_4 + r21_5 
            #print(r21)
            
            ######### twentytwo ###########
            twentytwo = self.pos_three_stage[93][0] - self.pos_three_stage[78][0]
            #print("twentytwo:", twentytwo)
            
            ########### segment1 ################### 
            twentytwo_1 = self.pos_three_stage[93][0] - self.pos_three_stage[504][0]
            #print("twentytwo_1:", twentytwo_1)
            
            epsilon_22_1 = ((  twentytwo_1  - 1.915390014649006)/ 1.915390014649006)*100
            r22_1 = ((R_spiral_2 * 1.915390014649006)/9.26638031005902)/2711.890962292 *( 40.9786262984012 * epsilon_22_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            twentytwo_2 = self.pos_three_stage[504][0] - self.pos_three_stage[904][0]
            #print("twentytwo_2:", twentytwo_2)
            
            epsilon_22_2 = (( twentytwo_2  - 1.8366661071777486)/ 1.8366661071777486)*100
            r22_2 = ((R_spiral_2 * 1.8366661071777486)/9.26638031005902)/2711.890962292 *( 40.9786262984012 * epsilon_22_2 + 2711.890962292 )
            
            
            ########### segment3 ################### 
            twentytwo_3 = self.pos_three_stage[904][0] - self.pos_three_stage[3252][0]
            #print("twentytwo_3  is :", twentytwo_3 ) 
            
            epsilon_22_3 = ((  twentytwo_3  - 1.8017539978038997)/ 1.8017539978038997)*100
            r22_3 = ((R_spiral_2 * 1.8017539978038997)/9.26638031005902)/2711.890962292 *( 40.9786262984012 * epsilon_22_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentytwo_4 = self.pos_three_stage[3252][0] - self.pos_three_stage[1107][0]
            #print("twentytwo_4 is :", twentytwo_4) 
            
            epsilon_22_4 = (( twentytwo_4  - 1.8406524658187777)/1.8406524658187777)*100
            r22_4 = ((R_spiral_2 * 1.8406524658187777)/9.26638031005902)/2711.890962292 *( 40.9786262984012 * epsilon_22_4 + 2711.890962292 )
          
          
            ########### segment5 ################### 
            twentytwo_5 = self.pos_three_stage[1107][0] - self.pos_three_stage[78][0]
            #print("twentytwo_5 is :", twentytwo_5) 
            
            epsilon_22_5 = ((  twentytwo_5  - 1.8719177246095882)/ 1.8719177246095882)*100
            r22_5 = ((R_spiral_2 * 1.8719177246095882)/9.26638031005902)/2711.890962292 *( 40.9786262984012 * epsilon_22_5 + 2711.890962292 )
            
            r22 = r22_1 + r22_2 + r22_3 + r22_4 + r22_5
            
            #print(r22)
            
            
            
            ################## Spirals 3 #########################
            
            
            
            
             ######### thirtyfive ###########
            thirtyfive = self.pos_three_stage[296][0] - self.pos_three_stage[302][0]
            #print(" thirtyfive is :",  thirtyfive)
            
            ######### segment 1 ###########
           
            thirtyfive_1 = self.pos_three_stage[296][0] - self.pos_three_stage[1997][0]
            #print(" thirtyfive_1 is :",  thirtyfive_1)
            
            epsilon_35_1 = ((thirtyfive_1 -  0.9003295898440058)/ 0.9003295898440058)*100
            r35_1 =  ((R_spiral_3 * 0.9003295898440058)/4.663085937500007)/2711.890962292 *( 40.9786262984012 * epsilon_35_1 + 2711.890962292 )
            
            
            ######### segment 2 ###########
           
            thirtyfive_2 = self.pos_three_stage[1997][0] - self.pos_three_stage[5528][0]
            #print(" thirtyfive_2 is :",  thirtyfive_2)
            
            epsilon_35_2 = ((thirtyfive_2 - 0.8719520568835648)/0.8719520568835648)*100
            r35_2 = ((R_spiral_3 * 0.8719520568835648)/4.663085937500007) /2711.890962292 *( 40.9786262984012 * epsilon_35_2 + 2711.890962292 )
            
            ######### segment 3 ###########
           
            thirtyfive_3 = self.pos_three_stage[5528][0] - self.pos_three_stage[2611][0]
            #print(" thirtyfive_3 is :",  thirtyfive_3)
            
            epsilon_35_3 = ((thirtyfive_3 - 0.9882507324229408)/0.9882507324229408)*100
            r35_3 = ((R_spiral_3 * 0.9882507324229408)/4.663085937500007) /2711.890962292 *( 40.9786262984012 * epsilon_35_3 + 2711.890962292 )
            
            ######### segment 4 ###########
           
            thirtyfive_4 = self.pos_three_stage[2611][0] - self.pos_three_stage[1126][0]
            #print(" thirtyfive_4 is :",  thirtyfive_4)
            
            epsilon_35_4 = ((thirtyfive_4 - 0.9496841430668468)/0.9496841430668468)*100
            r35_4 = ((R_spiral_3 * 0.9496841430668468)/4.663085937500007)/2711.890962292 *( 40.9786262984012 * epsilon_35_4 + 2711.890962292 )
            
        
            ######### segment 5 ###########
           
            thirtyfive_5 = self.pos_three_stage[1126][0] - self.pos_three_stage[302][0]
            #print(" thirtyfive_5 is :",  thirtyfive_5)
            
            epsilon_35_5 = ((thirtyfive_5 - 0.9528694152826489)/0.9528694152826489)*100
            r35_5 = ((R_spiral_3 * 0.9528694152826489)/4.663085937500007)/2711.890962292 *( 40.9786262984012 * epsilon_35_5 + 2711.890962292 )
            
            r35 = r35_1 + r35_2 + r35_3 + r35_4 + r35_5
            #print (r35)
            
            
             ######### thirtysix ###########
            thirtysix = self.pos_three_stage[321][1] - self.pos_three_stage[315][1]
            #print("thirtysix is :", thirtysix)
            
                        
            ########### segment 1 #####################
            thirtysix_1 = self.pos_three_stage[321][1] - self.pos_three_stage[5102][1]
            #print("thirtysix_1is :", thirtysix_1) 
            
            epsilon_36_1 = ((thirtysix_1 - 0.9598579406699912)/ 0.9598579406699912)*100
            
            r36_1 = ((R_spiral_3 * 0.9598579406699912)/4.695816040029996)/2711.890962292 *( 40.9786262984012 * epsilon_36_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            thirtysix_2 = self.pos_three_stage[5102][1] - self.pos_three_stage[5095][1]
            #print(" thirtysix_2is :",  thirtysix_2) 
            
            epsilon_36_2 = ((thirtysix_2 - 0.9922142028800067 )/ 0.9922142028800067)*100
            
            r36_2 = ((R_spiral_3 * 0.9922142028800067)/4.695816040029996)/2711.890962292 *( 40.9786262984012 * epsilon_36_2 + 2711.890962292 )
            
            ########### segment 3 #####################
            thirtysix_3 = self.pos_three_stage[5095][1] - self.pos_three_stage[5615][1]
            #print("thirtysix_3  is :", thirtysix_3 )
            
            epsilon_36_3 = ((thirtysix_3 - 0.9488048553450028 )/ 0.9488048553450028)*100
            
            r36_3 = ((R_spiral_3 * 0.9488048553450028)/4.695816040029996)/2711.890962292 *( 40.9786262984012 * epsilon_36_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            thirtysix_4 = self.pos_three_stage[5615][1] - self.pos_three_stage[4202][1]
            #print("thirtysix_4 is :", thirtysix_4) 
            
            epsilon_36_4 = ((thirtysix_4 - 0.8416175842277767 )/0.8416175842277767)*100
            
            r36_4 = ((R_spiral_3 * 0.8416175842277767)/4.695816040029996)/2711.890962292 *( 40.9786262984012 * epsilon_36_4 + 2711.890962292 )
            
            ########### segment 4 #####################
            thirtysix_5 = self.pos_three_stage[4202][1] - self.pos_three_stage[315][1]
            #print("thirtysix_5 is :", thirtysix_5) 
            
            epsilon_36_5 = ((thirtysix_5 -  0.9533214569072186 )/ 0.9533214569072186)*100
            
            r36_5 = ((R_spiral_3 * 0.9533214569072186 )/4.695816040029996)/2711.890962292 *( 40.9786262984012 * epsilon_36_5 + 2711.890962292 )
            
            r36 = r36_1 + r36_2 + r36_3 + r36_4 + r36_5
            #print(r36)
            
             ######### thirtyone ###########
            thirtyone = self.pos_three_stage[235][0] - self.pos_three_stage[249][0]
            #print("thirtyone is :", thirtyone)
            
             ########### segment 1 #####################
            thirtyone_1 = self.pos_three_stage[235][0] - self.pos_three_stage[234][0]
            #print("thirtyone_1 is :", thirtyone_1)
            
            epsilon_31_1 = (( thirtyone_1 - 0.897567749022997)/ 0.897567749022997)*100
            r31_1 = ((R_spiral_3 * 0.897567749022997)/4.620529174803998)/2711.890962292 *( 40.9786262984012 * epsilon_31_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            thirtyone_2 = self.pos_three_stage[234][0] - self.pos_three_stage[253][0]
            #print("thirtyone_2 is :", thirtyone_2)
            
            epsilon_31_2 = ((thirtyone_2 - 0.866653442382983)/ 0.866653442382983)*100
            r31_2 = ((R_spiral_3 * 0.866653442382983)/4.620529174803998)/2711.890962292 *( 40.9786262984012 * epsilon_31_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            thirtyone_3 = self.pos_three_stage[253][0] - self.pos_three_stage[2542][0]
            #print("thirtyone_3 is :", thirtyone_3)
            
            epsilon_31_3 = ((thirtyone_3 - 0.9870071411135086)/0.9870071411135086)*100
            r31_3 = ((R_spiral_3 * 0.9870071411135086)/4.620529174803998)/2711.890962292 *( 40.9786262984012 * epsilon_31_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            thirtyone_4 = self.pos_three_stage[2542][0] - self.pos_three_stage[1068][0]
            #print("thirtyone_4 is :", thirtyone_4)
            
            epsilon_31_4 = ((thirtyone_4 - 0.9322433471676277)/ 0.9322433471676277)*100
            r31_4 = ((R_spiral_3 * 0.9322433471676277)/4.620529174803998)/2711.890962292 *( 40.9786262984012 * epsilon_31_4 + 2711.890962292 )
            
            ########### segment 5 #####################
            thirtyone_5 = self.pos_three_stage[1068][0] - self.pos_three_stage[249][0]
            #print("thirtyone_5 is :", thirtyone_5)
            
            epsilon_31_5 = ((thirtyone_5 -  0.937057495116882 )/  0.937057495116882)*100
            
            r31_5 = ((R_spiral_3 * 0.937057495116882)/4.620529174803998)/2711.890962292 *( 40.9786262984012 * epsilon_31_5 + 2711.890962292 )
            
            r31 = r31_1 + r31_2 + r31_3 + r31_4 + r31_5
            #print(r31)
            
            
            ######### thirtytwo  ###########
            thirtytwo = self.pos_three_stage[260][0] - self.pos_three_stage[254][0]
            #print("thirtytwo is :", thirtytwo)
            
                        
            ########### segment1  #################### 
            thirtytwo_1 = self.pos_three_stage[260][0] - self.pos_three_stage[1076][0]
            #print("thirtytwo_1 is :", thirtytwo_1)
            
            epsilon_32_1 = (( thirtytwo_1   - 0.9523658752437569)/ 0.9523658752437569)*100
            r32_1 = ((R_spiral_3 * 0.9523658752437569)/4.6630859375)/2711.890962292 *( 40.9786262984012 *epsilon_32_1 + 2711.890962292 )
    
            
            ########### segment2  #################### 
            thirtytwo_2 = self.pos_three_stage[1076][0] - self.pos_three_stage[3131][0]
            #print("thirtytwo_2 is :", thirtytwo_2) 
            
            epsilon_32_2 = (( thirtytwo_2   - 0.950180053711243)/ 0.950180053711243)*100
            r32_2 = ((R_spiral_3 * 0.950180053711243)/4.6630859375)/2711.890962292 *( 40.9786262984012 * epsilon_32_2 + 2711.890962292 )
    
                         
            ########### segment3  #################### 
            thirtytwo_3 = self.pos_three_stage[3131][0] - self.pos_three_stage[256][0]
            #print("thirtytwo_3 is :", thirtytwo_3) 
            
            epsilon_32_3 = (( thirtytwo_3   - 0.9879112243649999)/ 0.9879112243649999)*100
            r32_3 = ((R_spiral_3 * 0.9879112243649999)/4.6630859375)/2711.890962292 *( 40.9786262984012 * epsilon_32_3 + 2711.890962292 )
    
            
            ########### segment4  #################### 
            thirtytwo_4 = self.pos_three_stage[256][0] - self.pos_three_stage[1886][0]
            #print("thirtytwo_4 is :", thirtytwo_4) 
            
            epsilon_32_4 = (( thirtytwo_4   - 0.8722763061530046)/ 0.8722763061530046)*100
            r32_4 = ((R_spiral_3 * 0.8722763061530046)/4.6630859375)/2711.890962292 *( 40.9786262984012 * epsilon_32_4 + 2711.890962292 )
            
            ########### segment5 #################### 
            thirtytwo_5 = self.pos_three_stage[1886][0] - self.pos_three_stage[254][0]
            #print("thirtytwo_5 is :", thirtytwo_5) 
            
            epsilon_32_5 = (( thirtytwo_5   - 0.9003524780269956)/ 0.9003524780269956)*100
            r32_5 = ((R_spiral_3 * 0.9003524780269956)/4.6630859375)/2711.890962292 *( 40.9786262984012 * epsilon_32_5 + 2711.890962292 )
            
            r32 = r32_1 + r32_2 + r32_3 + r32_4 + r32_5
            #print(r32)
            
              ######### thirtythree ###########
            thirtythree = self.pos_three_stage[347][1] - self.pos_three_stage[353][1]
            #print("thirtythree is :", thirtythree)
            
            ########### segment1 ################### 
            thirtythree_1 = self.pos_three_stage[347][1] - self.pos_three_stage[4111][1]
            #print("thirtythree_1  is :", thirtythree_1) 
            
            epsilon_33_1 = (( thirtythree_1   - 0.9535713195800923)/ 0.9535713195800923)*100
            r33_1 = ((R_spiral_3 * 0.9535713195800923)/4.6958160400389914)/2711.890962292 *( 40.9786262984012 *  epsilon_33_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            thirtythree_2 = self.pos_three_stage[4111][1] - self.pos_three_stage[1641][1]
            #print("thirtythree_2  is :", thirtythree_2) 
            
            epsilon_33_2 = (( thirtythree_2   - 0.9892082214399096)/ 0.9892082214399096)*100
            r33_2 = ((R_spiral_3 * 0.9892082214399096)/4.6958160400389914)/2711.890962292 *( 40.9786262984012 *  epsilon_33_2 + 2711.890962292 )
            
                        
            ########### segment3 ################### 
            thirtythree_3 = self.pos_three_stage[1641][1] - self.pos_three_stage[5598][1]
            #print("thirtythree_3  is :", thirtythree_3) 
            
            epsilon_33_3 = (( thirtythree_3   - 0.9431133270274898)/0.9431133270274898)*100
            r33_3 = ((R_spiral_3 * 0.9431133270274898)/4.6958160400389914)/2711.890962292 *( 40.9786262984012 *  epsilon_33_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            thirtythree_4 = self.pos_three_stage[5598][1] - self.pos_three_stage[762][1]
            #print("thirtythree_4  is :", thirtythree_4) 
            
            epsilon_33_4 = (( thirtythree_4   - 0.8172168731634883)/ 0.8172168731634883)*100
            r33_4 = ((R_spiral_3 * 0.8172168731634883)/4.6958160400389914)/2711.890962292 *( 40.9786262984012 *  epsilon_33_4 + 2711.890962292 )
            
            ########### segment5 ################### 
            thirtythree_5 = self.pos_three_stage[762][1] - self.pos_three_stage[353][1]
            #print("thirtythree_5  is :", thirtythree_5) 
            
            epsilon_33_5 = ((thirtythree_5   - 0.9927062988280113)/ 0.9927062988280113)*100
            r33_5 = ((R_spiral_3 * 0.9927062988280113)/4.6958160400389914)/2711.890962292 *( 40.9786262984012 *  epsilon_33_5 + 2711.890962292 )
            
            r33 = r33_1 + r33_2 + r33_3 + r33_4 + r33_5
            #print(r33)
            
            ######### thirtyfour ###########
            thirtyfour = self.pos_three_stage[275][0] - self.pos_three_stage[281][0]
            #print("thirtyfour is :", thirtyfour)
            
            ########### segment1 ################### 
            thirtyfour_1 = self.pos_three_stage[275][0] - self.pos_three_stage[3151][0]
            #print("thirtyfour_1  is :", thirtyfour_1 )
        
            epsilon_34_1 = (( thirtyfour_1 - 0.8792648315435017)/ 0.8792648315435017)*100
            r34_1 = ((R_spiral_3 * 0.8792648315435017)/4.620529174805) /2711.890962292 *( 40.9786262984012 * epsilon_34_1 + 2711.890962292 )
            
            
            ########### segment2 ################### 
            thirtyfour_2 = self.pos_three_stage[3151][0] - self.pos_three_stage[6241][0]
            #print("thirtyfour_2 is :", thirtyfour_2)
        
            epsilon_34_2 = ((thirtyfour_2 - 0.9900283813480115)/ 0.9900283813480115)*100
            r34_2 = ((R_spiral_3 * 0.9900283813480115)/4.620529174805) /2711.890962292 *( 40.9786262984012 * epsilon_34_2 + 2711.890962292 )
           
           
            ########### segment3 ################### 
            thirtyfour_3 = self.pos_three_stage[6241][0] - self.pos_three_stage[279][0]
            #print("thirtyfour_3 is :", thirtyfour_3)
        
            epsilon_34_3 = (( thirtyfour_3 - 0.9860076904294957)/ 0.9860076904294957)*100
            r34_3 =  ((R_spiral_3 * 0.9860076904294957)/4.620529174805)/2711.890962292 *( 40.9786262984012 * epsilon_34_3 + 2711.890962292 )
            
            
            ########### segment4 ################### 
            thirtyfour_4 = self.pos_three_stage[279][0] - self.pos_three_stage[280][0]
            #print(" thirtyfour_4 is :", thirtyfour_4)
        
            epsilon_34_4 = (( thirtyfour_4 - 0.8676376342769885)/ 0.8676376342769885)*100
            r34_4 = ((R_spiral_3 * 0.8676376342769885)/4.620529174805) /2711.890962292 *( 40.9786262984012 * epsilon_34_4 + 2711.890962292 )
            
            
            ########### segment5 ################### 
            thirtyfour_5 = self.pos_three_stage[280][0] - self.pos_three_stage[281][0]
            #print("thirtyfour_5 is :", thirtyfour_5)
        
            epsilon_34_5 = (( thirtyfour_5 - 0.8975906372070028)/ 0.8975906372070028)*100
            r34_5 = ((R_spiral_3 * 0.8975906372070028)/4.620529174805)/2711.890962292 *( 40.9786262984012 * epsilon_34_5 + 2711.890962292 )
            
            r34 = r34_1 + r34_2 + r34_3 + r34_4 + r34_5
            #print(r34)
    
          
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
            A[18][6] = r7 ; A[18][13] = r14 ; A[18][18] = -r19 ; A[18][12] = -r13 ; 								
            A[19][7] = r8 ; A[19][14] = r15 ; A[19][19] = -r20 ; A[19][13] = -r14 ; 								
            A[20][8] = r9 ; A[20][15] = r16 ; A[20][20] = -r21 ; A[20][14] = -r15 ; 								
            A[21][9] = r10 ; A[21][16] = r17 ; A[21][21] = -r22 ; A[21][15] = -r16 ; 								
            A[22][10] = r11 ; A[22][17] = r18 ; A[22][22] = -r23 ; A[22][16] = -r17 ; 								
            A[23][11] = r12 ; A[23][12] = r13 ; A[23][23] = -r24 ; A[23][17] = -r18 ; 								

	
########## Loop Second Stage ##############		
            A[24][18] = r19 ; A[24][25] = r26 ; A[24][30] = -r31 ; A[24][24] = -r25 ; 												
            A[25][19] = r20 ; A[25][26] = r27 ; A[25][31] = -r32 ; A[25][25] = -r26; 							
            A[26][20] = r21 ; A[26][27] = r28 ; A[26][32] = -r33 ; A[26][26] = -r27 ; 							
            A[27][21] = r22 ; A[27][28] = r29 ; A[27][33] = -r34 ; A[27][27] = -r28 ; 							
            A[28][22] = r23 ; A[28][29] = r30 ; A[28][34] = -r35 ; A[28][28] = -r29 ; 							
            A[29][23] = r24 ; A[29][24] = r25 ; A[29][35] = -r36 ; A[29][29] = -r30; 							


########## Loop Third Stage ##############
            A[30][30] = r31 ; A[30][37] = r38 ; A[30][36] = -r37 ; 
            A[31][31] = r32 ; A[31][38] = r39  ; A[31][37] = -r38 ; 						
            A[32][32] = r33 ; A[32][39] = r40  ; A[32][38] = -r39 ; 						
            A[33][33] = r34 ; A[33][40] = r41  ; A[33][39] = -r40 ; 						
            A[34][34] = r35 ; A[34][41] = r42  ; A[34][40] = -r41 ; 						
            A[35][35] = r36 ; A[35][36] = r37  ; A[35][41] = -r42 ; 						


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

            V = r7 * I[6] + r8 * I[7] +r9 * I[8];
######## check current is the same  I25 = I24 acual order 

            v_c = r13 * I[12] +  r25 * I[24] +  r37 * I[36] + r40 * I[39] + r28 * I[27] +  r16 * I[15] 
            
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

            V = r7 * I[6] ;
####### check current is the same 
            v_c = r12  * I[11] + r11 *I[10] + r10 * I[9] + r9 *  I[8] + r8 * I[7] 
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

            V = r7 * I[6] + r12 * I[11] + r11 * I[10]
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

            V = r11 * I[10] + r10 * I[9] + r9 * I[8]
            v_c = r12 * I[11] + r7 * I[6] + r8 * I[7]
            
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

            V = r9 * I[8]
            v_c =  r10 * I[9] + r11 * I[10] + r12 * I[11] +  r7 * I[6] +  r8 * I[7]
            
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

            V = r8 * I[7]
            v_c =  r7 * I[6] + r12 * I[11] + r11 * I[10] +  r10 * I[9] +  r9 * I[8]
            
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

            V = r12 * I[11]
            v_c =  r11 * I[10] + r10 * I[9] + r9 * I[8] +  r8 * I[7] +  r7 * I[6]
            
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

            V = r11 * I[10]
            v_c =  r12 * I[11] + r7 * I[6] + r8 * I[7] +  r9 * I[8] +  r10 * I[9]
            
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

            V = r10 * I[9]
            v_c =  r9 * I[8] + r8 * I[7] + r7 * I[6] +  r12 * I[11] +  r11 * I[10]
            
            R_t54 =  -1 * v_c
            R_t54 =  V
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
    three_stage.addObject('MeshVTKLoader', name='loader', filename=path + '3_stage_90.vtk', rotation=[0, 0, 0])
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
    three_stageVisu.addObject('MeshSTLLoader', filename=path + "3_stage_90.stl", name="loader")
    three_stageVisu.addObject('OglModel', src="@loader", color=[0.1, 0.1, 0.1, 0.9])
        
    three_stageVisu.addObject('TriangleCollisionModel')
    three_stageVisu.addObject('LineCollisionModel')
    three_stageVisu.addObject('PointCollisionModel')

    three_stageVisu.addObject('BarycentricMapping')


    return rootNode

