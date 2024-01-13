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
           
           
            ######### twentyfour ###########
            twentyfour = self.pos_three_stage[182][0] - self.pos_three_stage[86][0]
            #print("twentyfour is :", twentyfour)
            
            ########### segment 1 #####################
            twentyfour_1 = self.pos_three_stage[1012][0] - self.pos_three_stage[86][0]
            #print("twentyfour_1 is :", twentyfour_1)
                 
            epsilon_17 = ((twentyfour_1 -2.04049873358996 )/ 2.04049873358996)*100
            r17_1 = 399.342889501/2711.890962292 *( 40.9786262984012 * epsilon_7 + 2711.890962292 )
            
            
            ###### 29 ####
            
            twentyone = self.pos_three_stage[2590][0] - self.pos_three_stage[2354][0]
            #print("eight is :", eight)
            
            epsilon_29 = ((twentyone - 4.694999694824986)/4.694999694824986)*100
            r29 = 501.582467715/2711.890962292 *( 40.9786262984012 * epsilon_8 + 2711.890962292 )
            
            #print(r8)
        
            
               ###### 41 ####
            
            fourtyone = self.pos_three_stage[2659][0] - self.pos_three_stage[2596][0]
            #print("nine is :", nine)
            
            epsilon_41 = ((fourtyone - 4.01600265503)/4.01600265503)*100
            
            #print(epsilon_41)
            
            r41 = 429.042950599/2711.890962292 *( 40.9786262984012 * epsilon_41 + 2711.890962292 )
            #print(r9)
            
        
            ###### 38 ####
            
            
            thirtyeight = self.pos_three_stage[2563][0] - self.pos_three_stage[2661][0]
            #print("ten is :", ten)
            
            epsilon_38 = ((thirtyeight - 4.07999938963998)/4.07999938963998)*100
            
            #print(epsilon_10)
            
            r38 = 428.828993778/2711.890962292 *( 40.9786262984012 * epsilon_41 + 2711.890962292 )
            #print(r10)
            
            
            ###### 26 ####
            
                    
            twentysix = self.pos_three_stage[2396][0] - self.pos_three_stage[2530][0]
            #print("eleven is :", eleven)
            
            
            epsilon_26 = ((twentysix  - 4.093997955322003)/4.093997955322003)*100
            #print(epsilon_11)
            
            r26 = 437.375448495 /2711.890962292 *( 40.9786262984012 * epsilon_26 + 2711.890962292 )
            
            #print(r11)
            
            
            
            ###### 14 ####
            
            nineteen = self.pos_three_stage[2232][0] - self.pos_three_stage[2392][0]
            #print("twelve is :", twelve)
            
            epsilon_14 = ((twelve - 4.4599990844730115 )/ 4.4599990844730115)*100
            #print(epsilon_12)
            
            
            r14 = 476.476568876/2711.890962292 *( 40.9786262984012 * epsilon_14 + 2711.890962292 )
            
            #print(r12)
            
            
            
            ###### 18 ####
            
                             
            ######### 18 ###########
            eighteen = self.pos_three_stage[209][1] - self.pos_three_stage[208][1]
            epsilon_18 = ((thirtyone - 4.41894934079999)/ 4.41894934079999)*100
            #print(epsilon_31)
            
            r18 = 471.348126605/2711.890962292 *( 40.9786262984012 * epsilon_18 + 2711.890962292 )
            
               
            ######### thirty ###########
            thirty  = self.pos_three_stage[322][1] - self.pos_three_stage[321][1]
            epsilon_30 = ((thirty - 4.049003601069984 )/4.049003601069984 )*100
            #print(epsilon_30)
            
            r30 = 432.56855176/2711.890962292 *( 40.9786262984012 * epsilon_30 + 2711.890962292 )
            #print(r32) 
            
              ######### 42 ###########
            fourtytwo = self.pos_three_stage[363][1] - self.pos_three_stage[362][1]
            epsilon_42 = ((fourtytwo - 3.970001220709989)/3.970001220709989)*100
            #print(epsilon_33)
            
            r42 = 424.128464463/2711.890962292 *( 40.9786262984012 * epsilon_42 + 2711.890962292 )
            #print(r33) 
            
               ######### 39 ###########
            thirtynine = self.pos_three_stage[395][1] - self.pos_three_stage[394][1]
            epsilon_39 = ((thirtynine - 3.968994140620012)/3.968994140620012)*100
            #print(epsilon_34)
            
            r39 = 424.02087442/2711.890962292 *( 40.9786262984012 * epsilon_39 + 2711.890962292 )
            #print(r34)
          
                    
            ######### 27 ###########
            twentyseven = self.pos_three_stage[254][1] - self.pos_three_stage[273][1]
            #print(" thirty_5 is :",  thirty_5) 
            epsilon_27 = ((twentyseven - 4.642997741699006)/4.642997741699006)*100
            #print(epsilon_35)
            
            r27 = 496.026925995/2711.890962292 *( 40.9786262984012 * epsilon_27 + 2711.890962292 )
            #print(r35)
    
              ######### 15 ###########
            fifteen = self.pos_three_stage[170][1] - self.pos_three_stage[169][1]
            #print(" thirty_6 is :",  thirty_6) 
            epsilon_15 = ((fifteen - 3.697998046874986)/3.697998046874986)*100
            #print(epsilon_36)
            
            r15 = 395.06946171/2711.890962292 *( 40.9786262984012 * epsilon_15 + 2711.890962292 )
            #print(r36)
            
            
            
            ###### seven ####
            
            seven= self.pos_three_stage[2345][1] - self.pos_three_stage[2172][1]
            #print("one is :", one)
            
            epsilon_7 = ((seven - 4.41150283874496)/4.41150283874496)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r7= 471.295558183/2711.890962292 *( 40.9786262984012 * epsilon_7 + 2711.890962292 )
            #print(r1)
            
                 
             ###### ten ####
            
            ten = self.pos_three_stage[267][1] - self.pos_three_stage[2347][1]
            #print("two is :", two)
            
            epsilon_25 = ((ten - 4.049495697021499)/4.049495697021499)*100
            
            #print(epsilon_2 * 100)
            
            r25  = 432.621127927/2711.890962292 *( 40.9786262984012 * epsilon_25 + 2711.890962292 )
            #print(r2)
            
            
                 ###### thirtyseven ####
            
            thirtyseven = self.pos_three_stage[2643][1] - self.pos_three_stage[2615][1]
            #print("three is :", three)
            
            epsilon_37 = ((thirtyseven - 3.969497680661007)/3.969497680661007)*100
            
            #print(epsilon_3)
            
            r37 = 424.074670186/2711.890962292 *( 40.9786262984012 * epsilon_37 + 2711.890962292 )
            #print(r3)
            
            
             ###### fourty ####
            
            fourty = self.pos_three_stage[2524][1] - self.pos_three_stage[2649][1]
            #print("four is :", four)
            epsilon_40 = ((fourty - 3.969501495365023)/3.969501495365023)*100
            #print(epsilon_4)
            
            r40 = 424.075077471/2711.890962292 *( 40.9786262984012 * epsilon_40 + 2711.890962292 )
            #print(r4)
            
            
                                                 
             ###### 28 ####
            
            twentyeight = self.pos_three_stage[2388][1] - self.pos_three_stage[2521][1]
            #print("five is :", five)
            epsilon_28 = ((five - 4.6425056457500204)/4.6425056457500204)*100
            #print(epsilon_5)
            
            r28 = 495.974355487/2711.890962292 *( 40.9786262984012 * epsilon_28 + 2711.890962292 )
            #print(r5)
                          
                          
               ###### sixteen ####
            
            sixteen = self.pos_three_stage[2259][1] - self.pos_three_stage[2385][1]
            #print("six is :", six)
            
            epsilon_16 = ((sixteen - 3.699007122550494)/3.699007122550494)*100
            #print(epsilon_6)
            
            r16 = 395.17664769/2711.890962292 *( 40.9786262984012 * epsilon_16 + 2711.890962292 )
            
            #print(r6) 
                   
                   
                   
        ############### Spirals 1 ##################################
        
         ######### eleven ###########
            eleven = self.pos_three_stage[83][1] - self.pos_three_stage[97][1]
            #print("twenty_8 is :", twenty_8)
            
            ########### segment1 ################### 
            
            eleven_1 = self.pos_three_stage[2210][1] - self.pos_three_stage[34][1]
            #print("twentyeight_1 is :", twentyeight_1)
            
            epsilon_11_1 = ((eleven_1 - 2.934501647948494)/2.934501647948494)*100
            r11_1 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_11_1 + 2711.890962292 )
            
                        
            ########### segment2 ################### 
            
            eleven_2 = self.pos_three_stage[165][1] - self.pos_three_stage[2210][1]
            #print("twentyeight_2 is :", twentyeight_2)
            
            epsilon_11_2 = ((eleven_2 - 2.738502502441534)/2.738502502441534)*100
            r11_2 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_11_2 + 2711.890962292 )
            
            
            ########### segment3 ################### 
            
            eleven_3 = self.pos_three_stage[974][1] - self.pos_three_stage[165][1]
            #print("twentyeight_3 is :", twentyeight_3)
            
            epsilon_11_3 = ((eleven_3 - 2.953498840332145)/ 2.953498840332145)*100
            r11_3 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * eleven_3 + 2711.890962292 )
            
            ########### segment 4 ################### 
            
            eleven_4 = self.pos_three_stage[160][1] - self.pos_three_stage[974][1]
            #print("twentyeight_4 is :", twentyeight_4)
            
            epsilon_11_4 = ((eleven_4 - 2.7504959106448297)/ 2.7504959106448297)*100
            r11_4 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_11_4 + 2711.890962292 )
            
             ########### segment 5 ################### 
            
            eleven_5 = self.pos_three_stage[46][1] - self.pos_three_stage[160][1]
            #print("twentyeight_5 is :", twentyeight_5)
            
            epsilon_11_5 = ((eleven_5 - 2.9589996337839892)/ 2.9589996337839892)*100
            r11_5 = R_spiral_1 /2711.890962292 *( 40.9786262984012 * epsilon_11_5 + 2711.890962292 )
            
            r11 = r11_1 + r11_2 + r11_3 + r11_4 + r11_5
            
            
             ######### 12 ###########
            twelve = self.pos_three_stage[33][0] - self.pos_three_stage[21][0]
            #print("sixteen is :", sixteen)
            
            ########### segment 1 #####################
            twelve_1 = self.pos_three_stage[827][0] - self.pos_three_stage[21][0]
            #print("sixteen_1 is :", sixteen_1)
            
            epsilon_12_1 = ((twelve_1 - 2.9815025329589915 )/2.9815025329589915)*100
            r12_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_12_1 + 2711.890962292 )

            
            ########### segment 2 #####################
            twelve_2 = self.pos_three_stage[3568][0] - self.pos_three_stage[827][0]
            #print("sixteen_2 is :", sixteen_2)
            
            epsilon_12_2 = ((twelve_2 - 2.801248550415501 )/ 2.801248550415501)*100
            r12_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_12_2 + 2711.890962292 )


            ########### segment 3 #####################
            twelve_3 = self.pos_three_stage[3367][0] - self.pos_three_stage[3568][0]
            #print("sixteen_3 is :", sixteen_3)
            
            epsilon_12_3 = ((twelve_3 - 2.99574858897049 )/2.99574858897049)*100
            r12_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_12_3 + 2711.890962292 )

            
            
            ########### segment 4 #####################
            twelve_4 = self.pos_three_stage[6046][0] - self.pos_three_stage[3367][0]
            #print("sixteen_4 is :", sixteen_4)
            
                        
            epsilon_12_4 = ((twelve_4 - 2.9347515106204582 )/ 2.9347515106204582)*100
            r12_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_12_4 + 2711.890962292 )

            
            ########### segment 4 #####################
            twelve_5 = self.pos_three_stage[33][0] - self.pos_three_stage[6046][0]
            #print("sixteen_5 is :", sixteen_5)
            
                        
            epsilon_12_5 = ((twelve_5 - 2.7827510833740092)/ 2.7827510833740092)*100
            r16_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_12_5 + 2711.890962292 )
            
            r12 =  r12_1 +  r12_2 +  r12_3 +  r12_4 +  r12_5
            
            
            
             
               ######### seven ###########
            seven = self.pos_three_stage[8][1] - self.pos_three_stage[20][1]
            #print("seven is :", seven)
            
            ########### segment 1 #####################
            
            seven_1 = self.pos_three_stage[2164][1] - self.pos_three_stage[20][1]
            
            #print("seven_1 is :", seven_1)
            
            epsilon_7_1 = ((seven_1 - 2.934501647948508 )/ 2.934501647948508)*100
    
            r7_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_7_1 + 2711.890962292 )
            
            
            ########### segment 2 #####################
            seven_2 = self.pos_three_stage[110][1] - self.pos_three_stage[2164][1]
            
            #print("seven_2 is :", seven_2)
            
            epsilon_7_2 = ((seven_2 - 2.7385025024414773 )/ 2.7385025024414773)*100
            r7_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_7_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            
            seven_3 = self.pos_three_stage[920][1] - self.pos_three_stage[110][1]
            
            #print("seven_3 is :", seven_3)
            
            epsilon_7_3 = ((seven_3 - 2.953498840332145)/ 2.953498840332145)*100
            r7_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_7_3 + 2711.890962292 )
            
            
            ########### segment 4 #####################
            
            seven_4 = self.pos_three_stage[115][1] - self.pos_three_stage[920][1]
            
            #print("seven_4 is :", seven_4)
            
            epsilon_7_4 = ((seven_4 - 2.750495910644858 )/ 2.750495910644858)*100
            
            r7_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_7_4 + 2711.890962292 )
            
            
            ########### segment 5 #####################
            
            seven_5 = self.pos_three_stage[8][1] - self.pos_three_stage[115][1]
            
            #print("seven_5 is :", seven_5)
            
            epsilon_7_5 = ((seven_5 - 2.958999633783975 )/ 2.958999633783975)*100
            r7_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_7_5 + 2711.890962292 )
            
            r7 = r7_1 + r7_2 + r7_3 + r7_4 + r7_5 
            
            
            
            ######### eight ###########
            eight = self.pos_three_stage[73][1] - self.pos_three_stage[7][1]
            #print("eight is :", eight)
            ########### segment 1 #####################
            eight_1 = self.pos_three_stage[5498][1] - self.pos_three_stage[7][1]
            #print("eight_1 is :", eight_1) 
                                    
            epsilon_8_1 = (( eight_1  - 2.500249862675105)/ 2.500249862675105)*100
            r8_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_8_1 + 2711.890962292 )
         
            ########### segment 2 #####################
            eight_2 = self.pos_three_stage[2][1] - self.pos_three_stage[5498][1]
            #print("eight_2 is :", eight_2) 
                                    
            epsilon_8_2 = (( eight_2  - 2.8717517852748955)/ 2.8717517852748955)*100
            r8_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_8_2 + 2711.890962292 )
        
            ########### segment 3 #####################
            eight_3 = self.pos_three_stage[5973][1] - self.pos_three_stage[908][1]
            #print("eight_3 is :", eight_3) 
            
            epsilon_8_3 = (( eight_3  -  2.8335018157919336)/  2.8335018157919336)*100
            r8_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_8_3 + 2711.890962292 )
         
            ########### segment 4 #####################
            eight_4 = self.pos_three_stage[2297][1] - self.pos_three_stage[5973][1]
            #print("eight_4 is :", eight_4) 
            
            epsilon_8_4 = (( eight_4  - 2.8584995269770843)/  2.8584995269770843)*100
            r8_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_8_4 + 2711.890962292 )
         
            ########### segment 4 #####################
            eight_5 = self.pos_three_stage[73][1] - self.pos_three_stage[2297][1]
            #print("eight_5 is :", eight_5) 
            
            epsilon_8_5 = (( eight_5  - 2.9345054626449922 )/ 2.9345054626449922)*100
            r8_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_8_5 + 2711.890962292 )
         
            r8 = r8_1 + r8_2 + r8_3 + r8_4 + r8_5
            
            
            ######### nine ###########
            nine = self.pos_three_stage[60][0] - self.pos_three_stage[72][0]
            #print("twenty_2 is :", twenty_2)
           
            ########### segment1 #################### 
            nine_1 = self.pos_three_stage[873][0] - self.pos_three_stage[72][0]
            #print("nine_1  is :", nine_1 ) 
            
            epsilon_9_1 = (( nine_1   - 2.981502532958295)/ 2.981502532958295)*100
            r9_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_9_1 + 2711.890962292 )
            
            ########### segment2 #################### 
            nine_2 = self.pos_three_stage[4374][0] - self.pos_three_stage[873][0]
            #print("nine_2  is :", nine_2 ) 
            
            epsilon_9_2 = (( nine_2   - 2.801248550415778)/ 2.801248550415778)*100
            r9_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_9_2 + 2711.890962292 )
            
                      
            ########### segment3 #################### 
            nine_3 = self.pos_three_stage[4755][0] - self.pos_three_stage[4374][0]
            #print("nine_3  is :", nine_3 ) 
            
            epsilon_9_3 = (( nine_3   - 2.9957485198969636)/ 2.9957485198969636)*100
            r9_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_9_3 + 2711.890962292 )
            
            ########### segment4 #################### 
            nine_4 = self.pos_three_stage[866][0] - self.pos_three_stage[4755][0]
            #print("nine_4  is :", nine_4) 
            
            epsilon_9_4 = (( nine_4   - 2.736000061035277)/2.736000061035277)*100
            r9_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_9_4 + 2711.890962292 )  
            
                        
            ########### segment5 #################### 
            nine_5 = self.pos_three_stage[60][0] - self.pos_three_stage[866][0]
            #print("nine_5  is :", nine_5) 
            
            epsilon_9_5 = (( nine_5   - 2.9815025329596807)/ 2.9815025329596807)*100
            r9_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 *  epsilon_9_5 + 2711.890962292 )                        
            
            r9 = r9_1 + r9_2 + r9_3 + r9_4 + r9_5
            
            
            
             ######### ten ###########
            ten = self.pos_three_stage[118][1] - self.pos_three_stage[106][1]
            #print("twenty_5 is :", twenty_5)
            
            ########### segment1 ################### 
            ten_1 = self.pos_three_stage[5005][1] - self.pos_three_stage[47][1]
            #print("ten_1 is :", ten_1) 
            
            epsilon_10_1 = (( ten_1  - 2.8257503509586854)/ 2.8257503509586854)*100
            r10_1 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_10_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            ten_2 = self.pos_three_stage[946][1] - self.pos_three_stage[5005][1]
            #print("ten_2 is :", ten_2) 
            
            epsilon_10_2 = (( ten_2  - 2.8837490081753003)/ 2.8837490081753003)*100
            r10_2 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_10_2 + 2711.890962292 )
            
            ########### segment3 ################### 
            ten_3 = self.pos_three_stage[5972][1] - self.pos_three_stage[946][1]
            #print("ten_3 is :", ten_3) 
            
            epsilon_10_3 = (( ten_3  - 2.833501815793909)/ 2.833501815793909)*100
            r10_3 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_10_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            ten_4 = self.pos_three_stage[2251][1] - self.pos_three_stage[5972][1]
            #print("ten_4 is :", ten_4)
            
            epsilon_10_4 = (( ten_4  - 2.8584995269771127)/ 2.8584995269771127)*100
            r10_4 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_10_4 + 2711.890962292 )
            
                        
            ########### segment5 ################### 
            ten_5 = self.pos_three_stage[59][1] - self.pos_three_stage[2251][1]
            #print("ten_5 is :", ten_5) 
            
            epsilon_10_5 = (( ten_5  - 2.934505462644978)/ 2.934505462644978)*100
            r10_5 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_10_5 + 2711.890962292 )
            
            r10 = r10_1 + r10_2 + r10_3 + r10_4 + r10_5
            
        
            
            
            
    ############### Spirals 2 ##################################
            
            
             ######### twentyone ###########
            twentythree = self.pos_three_stage[275][1] - self.pos_three_stage[283][1]
           
            
                        
            ######### segment 1 ###########
            twentythree_1 = self.pos_three_stage[1089][1] - self.pos_three_stage[170][1]
            #print("twentythree_1 is :", twentythree_1)
            
            epsilon_23_1 = ((twentythree_1 - 2.0315017700190197)/ 2.0315017700190197)*100
            r23_1 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_23_1 + 2711.890962292 )
            
                             
            ######### segment 2 ###########
            twentythree_2 = self.pos_three_stage[3984][1] - self.pos_three_stage[1089][1]
            #print("twentythree_2 is :", twentythree_2)
            
            epsilon_23_2 = ((twentythree_2 - 2.075000762939368)/ 2.075000762939368)*100
            r23_2 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_23_2 + 2711.890962292 )
            
            ######### segment 3 ###########
            twentythree_3 = self.pos_three_stage[3966][1] - self.pos_three_stage[3984][1]
            #print("twentyone_3 is :", twentyone_3)
            
            epsilon_23_3 = ((twentythree_3 - 1.9287509918219854)/ 1.9287509918219854)*100
            r23_3 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_23_3 + 2711.890962292 )
            
            ######### segment 4 ###########
            twentythree_4 = self.pos_three_stage[3955][1] - self.pos_three_stage[3966][1]
            #print("twentythree_4 is :", twentyone_4)
            
            epsilon_29_4 = ((twentythree_4 - 2.000497817992283)/ 2.000497817992283)*100
            r23_4 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_23_4 + 2711.890962292 )
            
            ######### segment 5 ###########
            twentythree_5 = self.pos_three_stage[156][1] - self.pos_three_stage[3955][1]
            #print("twentythree_5 is :", twentythree_5)
            
            epsilon_23_5 = ((twentythree_5 - 1.9932479858353531)/1.9932479858353531)*100
            r23_5 = R_spiral_2 /2711.890962292 *( 40.9786262984012 * epsilon_23_5 + 2711.890962292 )
            
            r23 = r23_1 + r23_2 + r23_3 + r23_4 + r23_5
            
            
              ######### twentyfour ###########
            twentyfour = self.pos_three_stage[182][0] - self.pos_three_stage[196][0]
            #print("twentyfour is :", twentyfour)
            
            ########### segment 1 #####################
            twentyfour_1 = self.pos_three_stage[1012][0] - self.pos_three_stage[196][0]
            #print("twentyfour_1 is :", twentyfour_1)
                 
            epsilon_24_1 = ((twentyfour_1 -2.040498733519996 )/ 2.040498733519996)*100
            r24_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_24_1 + 2711.890962292 )
            
            
            ########### segment 2 #####################
            twentyfour_2 = self.pos_three_stage[2466][0] - self.pos_three_stage[1012][0]
            #print("twentyfour_2 is :", twentyfour_2)
            
            epsilon_24_2 = ((twentyfour_2 -   1.8907163759048302 )/  1.8907163759048302)*100
            r24_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_24_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            twentyfour_3 = self.pos_three_stage[3487][0] - self.pos_three_stage[2466][0]
            #print("twentyfour_3 is :", twentyfour_3)
            
                 
            epsilon_24_3 = ((twentyfour_3 -  2.0415336469834244 )/2.0415336469834244)*100
            r24_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_24_3 + 2711.890962292 )
            
            
            ########### segment 4 #####################
            twentyfour_4 = self.pos_three_stage[4238][0] - self.pos_three_stage[3487][0]
            #print("twentyfour_4 is :", twentyfour_4)
            
                 
            epsilon_24_4 = ((twentyfour_4 - 1.8332509994507546 )/ 1.8332509994507546)*100
            r24_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_24_4 + 2711.890962292 )
            
            
            ########### segment 5 #####################
            twentyfour_5 = self.pos_three_stage[182][0] - self.pos_three_stage[4238][0]
            #print("twentyfour_5 is :", twentyfour_5)
            
            epsilon_24_5 = ((twentyfour_5 -  1.9749984741209943)/ 1.9749984741209943)*100
            r24_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_24_5 + 2711.890962292 )
            
            r24 = r24_1 + r24_2 + r24_3 + r24_4 + r24_5
            
                
                ######### nineteen ###########
            nineteen = self.pos_three_stage[119][1] - self.pos_three_stage[104][1]
            
            ########### segment 1 #####################
            nineteen_1 = self.pos_three_stage[129][1] - self.pos_three_stage[104][1]
           
            epsilon_19_1 = ((nineteen_1 - 1.7959976196279968)/ 1.7959976196279968)*100            
            r19_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_19_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            nineteen_2 = self.pos_three_stage[2372][1] - self.pos_three_stage[129][1]
            
            #print("nineteen_2 is :", nineteen_2)
            
            epsilon_19_2 = ((nineteen_2 - 1.9025001525885301 )/ 1.9025001525885301 )*100            
            r19_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_9_2 + 2711.890962292 )
            
            ########### segment 3 #####################
            nineteen_3 = self.pos_three_stage[3649][1] - self.pos_three_stage[2372][1]
            #print("nineteen_3 is :", nineteen_3)
            
            epsilon_19_3 = ((nineteen_3 - 1.8752498626710121 )/ 1.8752498626710121)*100            
            r19_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_19_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            nineteen_4 = self.pos_three_stage[530][1] - self.pos_three_stage[3649][1]
            #print("nineteen_4 is :", nineteen_4)
            
            epsilon_19_4 = ((nineteen_4 - 1.8102474212644637 )/1.8102474212644639)*100            
            r19_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_19_4 + 2711.890962292 )
            
                        
            ########### segment 5 #####################
            nineteen_5 = self.pos_three_stage[119][1] - self.pos_three_stage[530][1]
            #print("nineteen_5 is :", nineteen_5)
            
            epsilon_19_5 = ((nineteen_5 - 1.9319992065380234 )/ 1.9319992065380234)*100            
            r19_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_19_5 + 2711.890962292 )
            
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
            
            
            ######### twentyone ###########
            twentyone = self.pos_three_stage[222][0] - self.pos_three_stage[208][0]
            #print("twentyone is :", twenty_3)
            
            ########### segment1 #################### 
            twentyone_1 = self.pos_three_stage[5176][0] - self.pos_three_stage[208][0]
            #print("twentyone_1  is :", twentyone_1) 
            
            epsilon_21_1 = (( twentyone_1   - 1.9342489242550158)/1.9342489242550158)*100
            r21_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_21_1 + 2711.890962292 )   
            
            ########### segment2 ################### 
            twentyone_2 = self.pos_three_stage[2504][0] - self.pos_three_stage[5176][0]
            #print("twentyone_2  is :", twentyone_2) 
            
            epsilon_21_2 = (( twentyone_2   - 1.9969661436740154)/ 1.9969661436740154)*100
            r21_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_21_2 + 2711.890962292 )   
            
            ########### segment3 ################### 
            twentyone_3 = self.pos_three_stage[5309][0] - self.pos_three_stage[2504][0]
            #print("twentyone_3  is :", twentyone_3) 
            
            epsilon_21_3 = (( twentyone_3   - 1.9910328187286268)/ 1.9910328187286268)*100
            r21_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_21_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentyone_4 = self.pos_three_stage[1040][0] - self.pos_three_stage[5309][0]
            #print("twentyone_4  is :", twentyone_4) 
            
            epsilon_21_4 = (( twentyone_4   - 1.883751869201859)/ 1.883751869201859)*100
            r21_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_21_4 + 2711.890962292 )
            
            
            ########### segment5 ################### 
            twentyone_5 = self.pos_three_stage[222][0] - self.pos_three_stage[1040][0]
            #print("twentyone_5  is :", twentyone_5) 
            
            epsilon_21_5 = (( twentyone_5   - 1.974998474120497)/ 1.974998474120497)*100
            r21_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 *  epsilon_21_5 + 2711.890962292 )
            
            r21 = r21_1 + r21_2 + r21_3 + r21_4 + r21_5 
            
            
            ######### twentytwo ###########
            twentytwo = self.pos_three_stage[294][1] - self.pos_three_stage[302][1]
            
            
            ########### segment1 ################### 
            twentytwo_1 = self.pos_three_stage[245][1] - self.pos_three_stage[145][1]
            
            epsilon_22_1 = ((  twentytwo_1  - 2.0360031128000315)/ 2.0360031128000315)*100
            r22_1 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_22_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            twentytwo_2 = self.pos_three_stage[2117][1] - self.pos_three_stage[245][1]
            
            epsilon_22_2 = (( twentytwo_2  - 1.9912858140034047)/ 1.9912858140034047)*100
            r22_2 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_22_2 + 2711.890962292 )
            
            
            ########### segment3 ################### 
            twentytwo_3 = self.pos_three_stage[6120][1] - self.pos_three_stage[2117][1]
            #print("twentysix_3 is :", twentysix_3) 
            
            epsilon_22_3 = ((  twentytwo_3  - 1.884961591999371)/ 1.884961591999371)*100
            r22_3 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_22_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            twentytwo_4 = self.pos_three_stage[1055][1] - self.pos_three_stage[6120][1]
            #print("twentysix_4 is :", twentysix_4) 
            
            epsilon_22_4 = (( twentytwo_4  - 2.0852508544923296)/ 2.0852508544923296)*100
            r22_4 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_22_4 + 2711.890962292 )
          
          
            ########### segment5 ################### 
            twentytwo_5 = self.pos_three_stage[131][1] - self.pos_three_stage[1055][1]
            #print("twentysix_5 is :", twentysix_5) 
            
            epsilon_22_5 = ((  twentytwo_5  - 2.0304985046348776)/ 2.0304985046348776)*100
            r22_5 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_22_5 + 2711.890962292 )
            
            r22 = r22_1 + r22_2 + r22_3 + r22_4 + r22_5
            
            
            
            
            
            ################## Spirals 3 #########################
            
            
            
            
             ######### thirtyfive ###########
            thirtyfive = self.pos_three_stage[377][1] - self.pos_three_stage[371][1]
            #print(" thirty is :",  thirty)
            
            ######### segment 1 ###########
           
            thirtyfive_1 = self.pos_three_stage[656][1] - self.pos_three_stage[254][1]
            #print(" thirty_1 is :",  thirty_1)
            
            epsilon_35_1 = ((thirtyfive_1 - 0.8990020751960088)/0.8990020751960088)*100
            r35_1 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_35_1 + 2711.890962292 )
            
            
            ######### segment 2 ###########
           
            thirtyfive_2 = self.pos_three_stage[675][1] - self.pos_three_stage[656][1]
            #print(" thirty_2 is :",  thirty_2)
            
            epsilon_35_2 = ((thirtyfive_2 - 0.8700027465820028)/0.8700027465820028)*100
            r35_2 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_35_2 + 2711.890962292 )
            
            ######### segment 3 ###########
           
            thirtyfive_3 = self.pos_three_stage[2554][1] - self.pos_three_stage[675][1]
            #print(" thirty_3 is :",  thirty_3)
            
            epsilon_35_3 = ((thirtyfive_3 - 1.081622123718006)/1.081622123718006)*100
            r35_3 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_35_3 + 2711.890962292 )
            
            ######### segment 4 ###########
           
            thirtyfive_4 = self.pos_three_stage[1076][1] - self.pos_three_stage[2554][1]
            #print(" thirty_4 is :",  thirty_4)
            
            epsilon_35_4 = ((thirtyfive_4 - 0.8468751907334848)/0.8468751907334848)*100
            r35_4 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_35_4 + 2711.890962292 )
            
        
            ######### segment 5 ###########
           
            thirtyfive_5 = self.pos_three_stage[260][1] - self.pos_three_stage[1076][1]
            #print(" thirty_5 is :",  thirty_5)
            
            epsilon_35_5 = ((thirtyfive_5 - 0.9444961547815325)/0.9444961547815325)*100
            r35_5 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_35_5 + 2711.890962292 )
            
            r35 = r35_1 + r35_2 + r35_3 + r35_4 + r35_5
            
            
            
             ######### thirtysix ###########
            thirtysix = self.pos_three_stage[353][0] - self.pos_three_stage[347][0]
            #print("eighteen is :", eighteen)
            
                        
            ########### segment 1 #####################
            thirtysix_1 = self.pos_three_stage[4158][0] - self.pos_three_stage[347][0]
            #print("eighteen_1 is :", eighteen_1) 
            
            epsilon_36_1 = ((thirtysix_1 - 0.9567489624021803 )/ 0.9567489624021803)*100
            
            r36_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_36_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            thirtysix_2 = self.pos_three_stage[5645][0] - self.pos_three_stage[4158][0]
            #print("eighteen_2 is :", eighteen_2) 
            
            epsilon_36_2 = ((thirtysix_2 - 0.8465013504033365 )/ 0.8465013504033365)*100
            
            r36_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_36_2 + 2711.890962292 )
            
            ########### segment 3 #####################
            thirtysix_3 = self.pos_three_stage[1184][0] - self.pos_three_stage[5645][0]
            #print("eighteen_3 is :", eighteen_3)
            
            epsilon_36_3 = ((thirtysix_3 - 0.9457483291624911 )/ 0.9457483291624911)*100
            
            r36_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_36_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            thirtysix_4 = self.pos_three_stage[4146][0] - self.pos_three_stage[1184][0]
            #print("eighteen_4 is :", eighteen_4) 
            
            epsilon_36_4 = ((thirtysix_4 - 0.9905014038088495 )/0.9905014038088495)*100
            
            r36_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_36_4 + 2711.890962292 )
            
            ########### segment 4 #####################
            thirtysix_5 = self.pos_three_stage[353][0] - self.pos_three_stage[4146][0]
            #print("eighteen_5 is :", eighteen_5) 
            
            epsilon_36_5 = ((thirtysix_5 -  0.956499099731154 )/ 0.956499099731154)*100
            
            r36_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_36_5 + 2711.890962292 )
            
            r36 = r36_1 + r36_2 + r36_3 + r36_4 + r36_5
            
            
             ######### thirtyone ###########
             thirtyone = self.pos_three_stage[295][1] - self.pos_three_stage[301][1]
            #print("fiveteen is :", fiveteen)
            
             ########### segment 1 #####################
            thirtyone_1 = self.pos_three_stage[300][1] - self.pos_three_stage[301][1]
            #print("fiveteen_1 is :", fiveteen_1)
            
            epsilon_31_1 = (( thirtyone_1 - 0.8990020751959662 )/ 0.8990020751959662)*100
            r31_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_31_1 + 2711.890962292 )
            
            ########### segment 2 #####################
            thirtyone_2 = self.pos_three_stage[1230][1] - self.pos_three_stage[300][1]
            #print("fiveteen_2 is :", fiveteen_2)
            
            epsilon_31_2 = ((thirtyone_2 - 0.9319992065425424)/ 0.9319992065425424)*100
            r31_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_31_2 + 2711.890962292 )
            
            
            ########### segment 3 #####################
            thirtyone_3 = self.pos_three_stage[2599][1] - self.pos_three_stage[1230][1]
            #print("fiveteen_3 is :", fiveteen_3)
            
            epsilon_31_3 = ((thirtyone_3 - 1.0196256637574095 )/1.0196256637574095)*100
            r31_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_31_3 + 2711.890962292 )
            
            ########### segment 4 #####################
            thirtyone_4 = self.pos_three_stage[385][1] - self.pos_three_stage[2599][1]
            #print("fiveteen_4 is :", fiveteen_4)
            
            epsilon_31_4 = ((thirtyone_4 - 0.7213716506950476)/ 0.7213716506950476)*100
            r31_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_31_4 + 2711.890962292 )
            
            ########### segment 5 #####################
            thirtyone_5 = self.pos_three_stage[295][1] - self.pos_three_stage[385][1]
            #print("fiveteen_5 is :", fiveteen_5)
            
            epsilon_31_5 = ((thirtyone_5 -  1.0699996948200123 )/  1.0699996948200123)*100
            
            r31_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_31_5 + 2711.890962292 )
            
            r31 = r31_1 + r31_2 + r31_3 + r31_4 + r31_5
            
            
            
            ######### thirtytwo  ###########
            thirtytwo = self.pos_three_stage[276][1] - self.pos_three_stage[282][1]
            #print("twenty_1 is :", twenty_1)
            
                        
            ########### segment1  #################### 
            thirtytwo_1 = self.pos_three_stage[1102][1] - self.pos_three_stage[282][1]
            #print("twentyone_1 is :", twentyone_1)
            
            epsilon_32_1 = (( thirtytwo_1   - 0.945003509525975)/ 0.945003509525975)*100
            r32_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 *epsilon_32_1 + 2711.890962292 )
    
            
            ########### segment2  #################### 
            thirtytwo_2 = self.pos_three_stage[3150][1] - self.pos_three_stage[1102][1]
            #print("twentyone_2 is :", twentyone_2) 
            
            epsilon_32_2 = (( thirtytwo_2   - 0.9409980773890254)/ 0.9409980773890254)*100
            r32_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_32_2 + 2711.890962292 )
    
                         
            ########### segment3  #################### 
            thirtytwo_3 = self.pos_three_stage[278][1] - self.pos_three_stage[3150][1]
            #print("twentyone_3 is :", twentyone_3) 
            
            epsilon_32_3 = (( thirtytwo_3   - 0.9879989624049728)/ 0.9879989624049728)*100
            r32_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_32_3 + 2711.890962292 )
    
            
            ########### segment4  #################### 
            thirtytwo_4 = self.pos_three_stage[691][1] - self.pos_three_stage[278][1]
            #print("twentyone_4 is :", twentyone_4) 
            
            epsilon_32_4 = (( thirtytwo_4   - 0.8690032959000149)/ 0.8690032959000149)*100
            r32_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_32_4 + 2711.890962292 )
            
            ########### segment5 #################### 
            thirtytwo_5 = self.pos_three_stage[276][1] - self.pos_three_stage[691][1]
            #print("twentyone_5 is :", twentyone_5) 
            
            epsilon_32_5 = (( thirtytwo_5   - 0.8989944458000139)/ 0.8989944458000139)*100
            r32_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_32_5 + 2711.890962292 )
            
            r32 = r32_1 + r32_2 + r32_3 + r32_4 + r32_5
            
            
              ######### thirtythree ###########
            thirtythree = self.pos_three_stage[315][0] - self.pos_three_stage[321][0]
            #print("twentyfour is :", twenty_4)
            
            ########### segment1 ################### 
            thirtythree_1 = self.pos_three_stage[364][0] - self.pos_three_stage[321][0]
            #print("twentyfour_1  is :", twentyfour_1) 
            
            epsilon_33_1 = (( thirtythree_1   - 0.9850006103519959)/ 0.9850006103519959)*100
            r33_1 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_33_1 + 2711.890962292 )
            
            ########### segment2 ################### 
            thirtythree_2 = self.pos_three_stage[5195][0] - self.pos_three_stage[364][0]
            #print("twentyfour_2  is :", twentyfour_2) 
            
            epsilon_33_2 = (( thirtythree_2   - 0.8182497024533291)/ 0.8182497024533291)*100
            r33_2 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_33_2 + 2711.890962292 )
            
                        
            ########### segment3 ################### 
            thirtythree_3 = self.pos_three_stage[1147][0] - self.pos_three_stage[5195][0]
            #print("twentyfour_3  is :", twentyfour_3) 
            
            epsilon_33_3 = (( thirtythree_3   - 0.9457483291624911)/ 0.9457483291624911)*100
            r33_3 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_33_3 + 2711.890962292 )
            
            ########### segment4 ################### 
            thirtythree_4 = self.pos_three_stage[2619][0] - self.pos_three_stage[1147][0]
            #print("thirtythree_4  is :", twentyfour_4) 
            
            epsilon_33_4 = (( thirtythree_4   - 1.0750007629396876)/ 1.0750007629396876)*100
            r33_4 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_33_4 + 2711.890962292 )
            
            ########### segment5 ################### 
            thirtythree_5 = self.pos_three_stage[315][0] - self.pos_three_stage[2619][0]
            #print("twentyfour_5  is :", twentyfour_5) 
            
            epsilon_33_5 = ((thirtythree_5   - 0.8719997406005007)/ 0.8719997406005007)*100
            r33_5 = R_spiral_3/2711.890962292 *( 40.9786262984012 *  epsilon_33_5 + 2711.890962292 )
            
            r33 = r33_1 + r33_2 + r33_3 + r33_4 + r33_5
            
            
            ######### thirtyfour ###########
            thirtyfour = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twenty_7 is :", twenty_7)
            
            ########### segment1 ################### 
            thirtyfour_1 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_1  is :", twentyseven_1 )
        
            epsilon_34_1 = (( thirtyfour_1 - 2.0304985046348776)/ 2.0304985046348776)*100
            r34_1 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_34_1 + 2711.890962292 )
            
            
            ########### segment2 ################### 
            thirtyfour_2 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_2 is :", twentyseven_2)
        
            epsilon_34_2 = ((thirtyfour_2 - 2.0304985046348776)/ 2.0304985046348776)*100
            r34_2 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_34_2 + 2711.890962292 )
           
           
            ########### segment3 ################### 
            thirtyfour_3 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_3 is :", twentyseven_3)
        
            epsilon_34_3 = (( twentyseven_3 - 2.0304985046348776)/ 2.0304985046348776)*100
            r34_3 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_34_3 + 2711.890962292 )
            
            
            ########### segment4 ################### 
            thirtyfour_4 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_4 is :", twentyseven_4)
        
            epsilon_34_4 = (( thirtyfour_4 - 2.0304985046348776)/ 2.0304985046348776)*100
            r34_4 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_34_4 + 2711.890962292 )
            
            
            ########### segment5 ################### 
            thirtyfour_5 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twentyseven_5 is :", twentyseven_5)
        
            epsilon_34_5 = (( thirtyfour_5 - 2.0304985046348776)/ 2.0304985046348776)*100
            r34_5 = R_spiral_3 /2711.890962292 *( 40.9786262984012 * epsilon_34_5 + 2711.890962292 )
            
            r34 = r34_1 + r34_2 + r34_3 + r34_4 + r34_5
         
    
          
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
    three_stage.addObject('MeshVTKLoader', name='loader', filename=path + '.vtk', rotation=[0, 0, 0])
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

