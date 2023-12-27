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
            
                     

#*********************** Radials    ************************#

              ###### one ####
            
            one = self.pos_three_stage[2345][1] - self.pos_three_stage[2172][1]
            #print("one is :", one)
            
            epsilon_1 = ((one - 4.411502838134496)/4.411502838134496)*100
            #print(epsilon_1)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r1= 471.295558183/2711.890962292 *( 40.9786262984012 * epsilon_1 + 2711.890962292 )
            #print(r1)
            
        
            
             ###### two ####
            
            two = self.pos_three_stage[2613][1] - self.pos_three_stage[2347][1]
            #print("two is :", two)
            
            epsilon_2 = ((two - 4.049495697021499)/4.049495697021499)*100
            
            #print(epsilon_2)
            
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
            
            epsilon_13 = ((thirteen - 14.335998535150964 )/ 14.335998535150964)*100
            #print(epsilon_13)
            
            r13 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_13 + 2711.890962292 )
            
            #print(r13)
            
            
                ######### fourteen ###########
            fourteen = self.pos_three_stage[119][1] - self.pos_three_stage[104][1]
            #print("fourteen is :", fourteen)
            
            epsilon_14 = ((fourteen - 9.315994262690026 )/ 9.315994262690026)*100
            #print(epsilon_14)
            
            
            r14 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_14 + 2711.890962292 )
            #print(r14)
            
                 ######### fiveteen ###########
            fiveteen = self.pos_three_stage[295][1] - self.pos_three_stage[301][1]
            #print("fiveteen is :", fiveteen)
            
            epsilon_15 = ((fiveteen - 4.641998291010978 )/ 4.641998291010978)*100
            #print(epsilon_15)
            
            r15 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_15 + 2711.890962292 )
            #print(r15)
            
            
            ######### sixteen ###########
            sixteen = self.pos_three_stage[33][0] - self.pos_three_stage[21][0]
            #print("sixteen is :", sixteen)
            epsilon_16 = ((sixteen - 14.496002197266009 )/ 14.496002197266009)*100
            #print(epsilon_16)
            
            
            r16 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_16 + 2711.890962292 )
            #print(r16)
            
            
            ######### seventeen ###########
            seventeen = self.pos_three_stage[182][0] - self.pos_three_stage[196][0]
            #print("seventeen is :", seventeen)
            epsilon_17 = ((seventeen -  9.78099822998 )/ 9.78099822998)*100
            
            #print(epsilon_17)
            
            r17 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_17 + 2711.890962292 )
            
            #print(r17)
                        
            ######### eighteen ###########
            eighteen = self.pos_three_stage[353][0] - self.pos_three_stage[347][0]
            #print("eighteen is :", eighteen)
            epsilon_18 = ((eighteen - 4.6959991455080115 )/ 4.6959991455080115)*100
            #print(epsilon_18)
            
            r18 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_18 + 2711.890962292 )
            #print(r18)
            
                                    
            ######### nineteen ###########
            nineteen = self.pos_three_stage[73][1] - self.pos_three_stage[7][1]
            #print("nineteen is :", nineteen)
            epsilon_19 = (( nineteen  - 14.33600616455 )/ 14.33600616455)*100
            #print(epsilon_19)
            
            r19 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_19 + 2711.890962292 )
            #print(r19)
            
            ######### twenty ###########
            twenty = self.pos_three_stage[84][1] - self.pos_three_stage[96][1]
            #print("twenty is :", twenty)
            epsilon_20 = (( twenty  - 9.31600189209 )/ 9.31600189209)*100
            #print(epsilon_20)
            
            r20 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_20 + 2711.890962292 )
            #print(r20)
            
                        
            ######### twenty_1 ###########
            twenty_1 = self.pos_three_stage[276][1] - self.pos_three_stage[282][1]
            #print("twenty_1 is :", twenty_1)
            
            epsilon_21 = ((twenty_1 - 4.641998291020002 )/ 4.641998291020002)*100
            
            #print(epsilon_21)
            
            r21 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_21 + 2711.890962292 )
            #print(r21)
            
                         
            ######### twenty_2 ###########
            twenty_2 = self.pos_three_stage[60][0] - self.pos_three_stage[72][0]
            #print("twenty_2 is :", twenty_2)
            epsilon_22 = ((twenty_2 - 14.496002197265994 )/ 14.496002197265994)*100
            #print(epsilon_22)
            
            r22 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_22 + 2711.890962292 )
            #print(r22)
                                     
            ######### twenty_3 ###########
            twenty_3 = self.pos_three_stage[222][0] - self.pos_three_stage[208][0]
            #print("twenty_3 is :", twenty_3)
            epsilon_23 = ((twenty_3 - 9.780998229980014)/ 9.780998229980014)*100
            #print(epsilon_23)
            
            r23 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_23 + 2711.890962292 )
            #print(r23)
            
                                                 
            ######### twenty_4 ###########
            twenty_4 = self.pos_three_stage[315][0] - self.pos_three_stage[321][0]
            #print("twenty_4 is :", twenty_4)
            epsilon_24 = ((twenty_4 - 4.695999145508004)/ 4.695999145508004)*100
            
            #print(epsilon_24)
            
            r24 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_24 + 2711.890962292 )
            #print(r24)
            
            
            ######### twenty_5 ###########
            twenty_5 = self.pos_three_stage[118][1] - self.pos_three_stage[106][1]
            #print("twenty_5 is :", twenty_5)
            epsilon_25 = ((twenty_5 - 13.726997375482995)/ 13.726997375482995)*100
            
            #print(epsilon_25)
            
            r25 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_25 + 2711.890962292 )
            #print(r25)
            
            
            ######### twenty_6 ###########
            twenty_6 = self.pos_three_stage[294][1] - self.pos_three_stage[302][1]
            #print("twenty_6 is :", twenty_6)
            epsilon_26 = ((twenty_6 - 8.691993713374018)/ 8.691993713374018)*100
            ##print(epsilon_26)
            
            r26 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_26 + 2711.890962292 )
            #print(r26)
            
            
            ######### twenty_7 ###########
            twenty_7 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            #print("twenty_7 is :", twenty_7)
            epsilon_27 = ((twenty_7 - 3.9689941406199694)/ 3.9689941406199694)*100
            #print(epsilon_27)
            
            r27 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_27 + 2711.890962292 )
            #print(r27)
            
            
            ######### twenty_8 ###########
            twenty_8 = self.pos_three_stage[83][1] - self.pos_three_stage[97][1]
            #print("twenty_8 is :", twenty_8)
            epsilon_28 = ((twenty_8 - 13.727005004890032)/ 13.727005004890032)*100
            #print(epsilon_28)
            
            r28 = R_spiral_1/2711.890962292 *( 40.9786262984012 * epsilon_28 + 2711.890962292 )
            #print(r28)
            
            
            ######### twenty_9 ###########
            twenty_9 = self.pos_three_stage[275][1] - self.pos_three_stage[283][1]
            #print("twenty_9 is :", twenty_9)
            epsilon_29 = ((twenty_9 - 8.692001342780017 )/ 8.692001342780017)*100
            #print(epsilon_29)
            
            r29 = R_spiral_2/2711.890962292 *( 40.9786262984012 * epsilon_29 + 2711.890962292 )
            #print(r29)
            
            
            ######### thirty ###########
            thirty = self.pos_three_stage[377][1] - self.pos_three_stage[371][1]
            #print(" thirty is :",  thirty)
            
            epsilon_30 = ((thirty - 3.969001770019986 )/ 3.969001770019986)*100
            #print(epsilon_30)
            
            r30 = R_spiral_3/2711.890962292 *( 40.9786262984012 * epsilon_30 + 2711.890962292 )
            #print(r30)
            
            
            
            ######### radials  ###########
            
            ######### thirty_1 ###########
            thirty_1 = self.pos_three_stage[209][1] - self.pos_three_stage[208][1]
            #print(" thirty_1 is :",  thirty_1) 
            epsilon_31 = ((thirty_1 - 4.411994934079999)/ 4.411994934079999)*100
            #print(epsilon_31)
            
            r31 = 471.348126605/2711.890962292 *( 40.9786262984012 * epsilon_31 + 2711.890962292 )
            
            #print(r31)  
            
            
            ######### thirty_2 ###########
            thirty_2 = self.pos_three_stage[322][1] - self.pos_three_stage[321][1]
            #print(" thirty_2 is :",  thirty_2) 
            epsilon_32 = ((thirty_2 - 4.049003601069984 )/4.049003601069984 )*100
            #print(epsilon_32)
            
            r32 = 432.56855176/2711.890962292 *( 40.9786262984012 * epsilon_32 + 2711.890962292 )
            #print(r32) 
            
            ######### thirty_3 ###########
            thirty_3 = self.pos_three_stage[363][1] - self.pos_three_stage[362][1]
            #print(" thirty_3 is :",  thirty_3) 
            epsilon_33 = ((thirty_3 - 3.970001220709989)/3.970001220709989)*100
            #print(epsilon_33)
            
            r33 = 424.128464463/2711.890962292 *( 40.9786262984012 * epsilon_33 + 2711.890962292 )
            #print(r33) 
                        
            ######### thirty_4 ###########
            thirty_4 = self.pos_three_stage[395][1] - self.pos_three_stage[394][1]
            #print(" thirty_4 is :",  thirty_4) 
            epsilon_34 = ((thirty_4 - 3.968994140620012)/3.968994140620012)*100
            #print(epsilon_34)
            
            r34 = 424.02087442/2711.890962292 *( 40.9786262984012 * epsilon_34 + 2711.890962292 )
            #print(r34)
            
            ######### thirty_5 ###########
            thirty_5 = self.pos_three_stage[254][1] - self.pos_three_stage[273][1]
            #print(" thirty_5 is :",  thirty_5) 
            epsilon_35 = ((thirty_5 - 4.642997741699006)/4.642997741699006)*100
            #print(epsilon_35)
            
            r35 = 496.026925995/2711.890962292 *( 40.9786262984012 * epsilon_35 + 2711.890962292 )
            #print(r35)
            
                        
            ######### thirty_6 ###########
            thirty_6 = self.pos_three_stage[170][1] - self.pos_three_stage[169][1]
            #print(" thirty_6 is :",  thirty_6) 
            epsilon_36 = ((thirty_6 - 3.697998046874986)/3.697998046874986)*100
            #print(epsilon_36)
            
            r36 = 395.06946171/2711.890962292 *( 40.9786262984012 * epsilon_36 + 2711.890962292 )
            #print(r36)
            
            ######************ Kirchoff equations ****########
            

############# Zeros the matrices of A.I = C ##############
#### Where A are the coefficients, I is the current, C are the constants, and R are the resistances 
            A = np.zeros((42,42))
            I = np.zeros((42,1))
            C = np.zeros((42,1))
            I[0] = 1;

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


########## Boundary Conditions 1 and 4 ############## 
            A[36][0] = 1 ; C[36][0] = 1 ;		
            A[37][1] = 1 ; C[37][0] = 0 ;		
            A[38][2] = 1 ; C[38][0] = 0 ;		
            A[39][3] = 1 ; C[39][0] = -1 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


# Solve the matrix equation
            I = solve(A, C)
            #print(I[2])

##### Access the solution######
####  I[7] is I_6 ########

            V = r13 * I[6] + r19* I[7] +r22 * I[8];
####### check current is the same 
            v_c = summation of radial elements   
            R_t14 = V/I[0];
            #print(V, I[0], R_t)

########## Boundary Conditions 1 and 2 ############## 
            A[36][0] = 1 ; C[36][0] = 1 ;		
            A[37][1] = 1 ; C[37][0] = -1 ;		
            A[38][2] = 1 ; C[38][0] = 0 ;		
            A[39][3] = 1 ; C[39][0] = 0 ;		
            A[40][4] = 1 ; C[40][0] = 0 ;		
            A[41][5] = 1 ; C[41][0] = 0 ;


# Solve the matrix equation
            I = solve(A, C);
            

##### Access the solution######
####  I[7] is I_6 ########

            V = r13 * I[6] ;
####### check current is the same 
            v_c = summation of outer spirals elements  (negative sign)   
            R_t12 = V/I[0];
            #print(V, I[0], R_t)

##current = I[10]
#print(current)


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
            matrix_ep = ((matrix_1 - 20.104896677387785) + (matrix_2 - 20.441054740343915 ) + (matrix_3 - 20.56515969337761 ) + (matrix_4 - 20.901317756333697 ) + (matrix_5 - 20.104896677387785 ))/ (20.104896677387785 + 20.441054740343915 + 20.56515969337761 + 20.901317756333697 + 20.104896677387785)   
            #print(matrix_ep)

            

                

	
		
		


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

