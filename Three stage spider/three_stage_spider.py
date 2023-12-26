9# -*- coding: utf-8 -*-

import Sofa

import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

################################ Paramters ##################################
R_radial = 3205  #one 
R_spiral_1  = 1731  # one 
R_spiral_2  = 1124  # one 
R_spiral_3  = 560.49 # one 


r_radial_segment = 641
r_spiral_segment_1 = 346.2
r_spiral_segment_2 = 224.9
r_spiral_segment_3 = 112.098

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
            
                     

#***********************  part1 in one stage "Radial parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_1 = self.pos_three_stage[60][1] - self.pos_three_stage[20][1]
            #print("three_stage_1 length is :", three_stage_1)
            
            
#######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_1 = self.pos_three_stage[1130][1] - self.pos_three_stage[20][1]
            #print("three_stage_1_segment_1 length is :", three_stage_1_segment_1) 
            
            
            epsilon_11 = ((three_stage_1_segment_1 - 6.062503814697024)/6.062503814697024)*100
            
            ##print(" epsilon_11 is :", epsilon_11) 
            
           
            
            r11 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_11 + 2711.890962292 )
          
            
            
            
            
            
            #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_2 = self.pos_three_stage[4140][1] - self.pos_three_stage[1130][1]
            #print("three_stage_1_segment_2 length is :", three_stage_1_segment_2) 
            
            
            epsilon_12 = ((three_stage_1_segment_2 - 5.7928733825667535)/5.7928733825667535)*100
            
                       
            r12 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_12 + 2711.890962292 )
          
            
    

            #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_3 = self.pos_three_stage[1693][1] - self.pos_three_stage[4140][1]
            #print("three_stage_1_segment_3 length is :", three_stage_1_segment_3) 
            
            
            epsilon_13 = ((three_stage_1_segment_3 - 5.955623626707009)/5.955623626707009)*100
                       
            r13 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_13 + 2711.890962292 )
          
            #print(" r_13 is :", r_13)
            
            
            
            #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_4 = self.pos_three_stage[1054][1] - self.pos_three_stage[1693][1]
            #print("three_stage_1_segment_4 length is :", three_stage_1_segment_4) 
            
            
            epsilon_14 = ((three_stage_1_segment_4 - 6.043502807615141)/6.043502807615141)*100
            
            #print(" r_14 is :", r_14)
                       
            r14 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_14 + 2711.890962292 )
          
            
	
        #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_5 = self.pos_three_stage[60][1] - self.pos_three_stage[1054][1]
            #print("three_stage_1_segment_5 length is :", three_stage_1_segment_5) 
            
            
            epsilon_15 = ((three_stage_1_segment_5 - 5.813495635985063)/5.813495635985063)*100
            
                       
            r15 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_15 + 2711.890962292 )
          
            #print(" r_15 is :", r_15)
            
                                    
   ######&&&&&&&&&&&&  R_ part1_ Summations &&&&&&&&&&&& #####
            
            r_1sum = r11 + r12 + r13 + r14 + r15 
            epsilon_1sum = epsilon_11 + epsilon_12 + epsilon_13 + epsilon_14 + epsilon_15
      
            #print(r_1sum )
            
            
                             

#***********************  part2 in one stage "Radial parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_2 = self.pos_three_stage[72][1] - self.pos_three_stage[33][1]
            #print("three_stage_2 length is :", three_stage_2)
            
            
#######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_1 = self.pos_three_stage[983][1] - self.pos_three_stage[33][1]
            #print("three_stage_2_segment_1 length is :", three_stage_2_segment_1) 
            
            
            epsilon_21 = ((three_stage_2_segment_1 - 6.034500122069957)/6.034500122069957)*100
                       
            r21 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_21 + 2711.890962292 )
          
            #print(" r_21 is :", r_21)
            
                        
#######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_2 = self.pos_three_stage[2552][1] - self.pos_three_stage[983][1]
            #print("three_stage_2_segment_2 length is :", three_stage_2_segment_2) 
            
            
            epsilon_22 = ((three_stage_2_segment_2 - 5.980503082275561)/5.980503082275561)*100
                       
            r22 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_22 + 2711.890962292 )
          
            #print(" r_22 is :", r_22)
            
                                    
#######&&&&----------------  part2 segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_3 = self.pos_three_stage[4571][1] - self.pos_three_stage[2552][1]
            #print("three_stage_2_segment_3 length is :", three_stage_2_segment_3) 
            
            
            epsilon_23 = ((three_stage_2_segment_3 - 6.127998352047484)/6.127998352047484)*100
                       
            r23 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_23 + 2711.890962292 )
          
            #print(" r_23 is :", r_23)
            
            
            #######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_4 = self.pos_three_stage[890][1] - self.pos_three_stage[4571][1]
            #print("three_stage_2_segment_4 length is :", three_stage_2_segment_4) 
            
            
            epsilon_24 = ((three_stage_2_segment_4 - 6.052001953120012)/6.052001953120012)*100
                       
            r24 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_24 + 2711.890962292 )
          
            
            
            #######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_5 = self.pos_three_stage[72][1] - self.pos_three_stage[890][1]
            #print("three_stage_2_segment_5 length is :", three_stage_2_segment_5) 
            
            
            epsilon_25 = ((three_stage_2_segment_5 - 5.8049964904799936)/5.8049964904799936)*100
        
                       
            r25 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_25 + 2711.890962292 )
          
            #print(" r_25 is :", r_25)
        
            r_2sum = r21 + r22 + r23 + r24 + r25 
            epsilon_2sum = epsilon_21 + epsilon_22 + epsilon_23 + epsilon_24 + epsilon_25
      
            print(r_2sum)
                        
                        
            
            
            
            
#***********************  part3 in one stage "Radial parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_3 = self.pos_three_stage[850][0] - self.pos_three_stage[811][0]
            #print("three_stage_3 length is :", three_stage_3)
            
            
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_1 = self.pos_three_stage[1108][0] - self.pos_three_stage[811][0]
            #print("three_stage_3_segment_1 length is :", three_stage_3_segment_1) 
            
            
            epsilon_31 = ((three_stage_3_segment_1 - 5.961999893187119)/5.961999893187119)*100
                       
            r31 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_31 + 2711.890962292 )
          
            #print(" r_31 is :", r_31)
            
                        
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_2 = self.pos_three_stage[785][0] - self.pos_three_stage[1108][0]
            #print("three_stage_3_segment_2 length is :", three_stage_3_segment_2) 
            
            
            epsilon_32 = ((three_stage_3_segment_2 - 5.924001693726879)/5.961999893187119)*100
        
                       
            r32 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_32 + 2711.890962292 )
          
            
            
                                 
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_3 = self.pos_three_stage[1240][0] - self.pos_three_stage[785][0]
            #print("three_stage_3_segment_3 length is :", three_stage_3_segment_3) 
            
            
            epsilon_33 = ((three_stage_3_segment_3 - 6.0789985656744605)/ 6.0789985656744605)*100

                       
            r33 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_33 + 2711.890962292 )
          
            
            
                                             
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_4 = self.pos_three_stage[1065][0] - self.pos_three_stage[1240][0]
            #print("three_stage_3_segment_4 length is :", three_stage_3_segment_4) 
            
            
            epsilon_34 = ((three_stage_3_segment_4 - 6.069000244141243)/ 6.069000244141243)*100
            
            r34 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_34 + 2711.890962292 )
          
            
                                                         
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_5 = self.pos_three_stage[850][0] - self.pos_three_stage[1065][0]
            #print("three_stage_3_segment_5 length is :", three_stage_3_segment_5) 
            
            
            epsilon_35 = ((three_stage_3_segment_5 - 5.965999603270298)/ 5.965999603270298)*100
        
                       
            r35 = r_radial_segment/2711.890962292 *( 40.9786262984012 * epsilon_35 + 2711.890962292 )
          
            
            
            r_3sum = r31 + r32 + r33 + r34 + r35 
            epsilon_3sum = epsilon_11 + epsilon_12 + epsilon_13 + epsilon_14 + epsilon_15
            #print(epsilon_3sum)
            
            #print(r_3sum)
                        
                        
                        
                  
            
#***********************  part4 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_4 = self.pos_three_stage[33][0] - self.pos_three_stage[21][0]
            #print("three_stage_4 length is :", three_stage_4)
            
            
########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_1 = self.pos_three_stage[827][0] - self.pos_three_stage[21][0]
            #print("three_stage_4_segment_1 length is :", three_stage_4_segment_1) 
            
            
            epsilon_41 = ((three_stage_4_segment_1 - 2.9815025329589915)/2.9815025329589915)*100
            
            r41 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_41 + 2711.890962292 )
          
            
                        
########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_2 = self.pos_three_stage[5457][0] - self.pos_three_stage[827][0]
            #print("three_stage_4_segment_2 length is :", three_stage_4_segment_2) 
            
            
            epsilon_42 = ((three_stage_4_segment_2 - 2.7757492065430185)/2.7757492065430185)*100
            
            r42 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_42 + 2711.890962292 )
          
            
            
                                    
########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_3 = self.pos_three_stage[3367][0] - self.pos_three_stage[3383][0]
            #print("three_stage_4_segment_3 length is :", three_stage_4_segment_3) 
            
            
            epsilon_43 = ((three_stage_4_segment_3 - 3.060247421264826)/3.060247421264826)*100
        
            r43 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_43 + 2711.890962292 )
          
                        
                        
    ########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_4 = self.pos_three_stage[834][0] - self.pos_three_stage[3367][0]
            #print("three_stage_4_segment_4 length is :", three_stage_4_segment_4) 
            
            
            epsilon_44 = ((three_stage_4_segment_4 - 2.7360000610354547)/ 2.7360000610354547)*100
            
            r44 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_44 + 2711.890962292 )
          
            
            
                                    
    ########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_5 = self.pos_three_stage[33][0] - self.pos_three_stage[834][0]
            #print("three_stage_4_segment_5 length is :", three_stage_4_segment_5) 
            
            
            epsilon_45 = ((three_stage_4_segment_5 - 2.981502532959013)/ 2.981502532959013)*100
    
            r45 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_45 + 2711.890962292 )
          
            
            r_4sum = r41 + r42 + r43 + r44 + r45 
            epsilon_4sum = epsilon_41 + epsilon_42 + epsilon_43 + epsilon_44 + epsilon_45
            #print(r_4sum)    
                        
                        
                        
                        
                 
#***********************  part5 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_5 = self.pos_three_stage[8][1] - self.pos_three_stage[20][1]
            #print("three_stage_5 length is :", three_stage_5)
            
            
########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_1 = self.pos_three_stage[2164][1] - self.pos_three_stage[20][1]
            #print("three_stage_5_segment_1 length is :", three_stage_5_segment_1) 
            
            
            epsilon_51 = ((three_stage_5_segment_1 - 2.934501647948508)/2.934501647948508)*100
        
                
            r51 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_51 + 2711.890962292 )
          
            
                
########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_2 = self.pos_three_stage[110][1] - self.pos_three_stage[2164][1]
            #print("three_stage_5_segment_2 length is :", three_stage_5_segment_2) 
            
            
            epsilon_52 = ((three_stage_5_segment_2 - 2.7385025024414773)/2.7385025024414773)*100

                
            r52 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_52 + 2711.890962292 )
          
                    
                        
                        
                        
    ########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_3 = self.pos_three_stage[2156][1] - self.pos_three_stage[110][1]
            #print("three_stage_5_segment_3 length is :", three_stage_5_segment_3) 
            
            
            epsilon_53 = ((three_stage_5_segment_3 - 2.8364982604980042)/2.8364982604980042)*100
                
            r53 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_53 + 2711.890962292 )
          
                    
                        
    ########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_4 = self.pos_three_stage[115][1] - self.pos_three_stage[2156][1]
            #print("three_stage_5_segment_4 length is :", three_stage_5_segment_4) 
            
            
            epsilon_54 = ((three_stage_5_segment_4 - 2.867496490478999)/2.867496490478999)*100
                
            r54 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_54 + 2711.890962292 )
          
                    
                    
                    
                                            
    ########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_5 = self.pos_three_stage[8][1] - self.pos_three_stage[115][1]
            #print("three_stage_5_segment_5 length is :", three_stage_5_segment_5) 
            
            
            epsilon_55 = ((three_stage_5_segment_5 - 2.958999633783975)/2.958999633783975)*100
                            
            r55 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_55 + 2711.890962292 )
          
                    
                    
                      
            r_5sum = r51 + r52 + r53 + r54 + r55 
            epsilon_5sum = epsilon_51 + epsilon_52 + epsilon_53 + epsilon_54 + epsilon_55
      
            #print(r_5sum)  
                    
                    
                    
                                   
#***********************  part6 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_6 = self.pos_three_stage[1971][1] - self.pos_three_stage[7][1]
            #print("three_stage_6 length is :", three_stage_6)
            
            
#########&&&&----------------  part6  segments  ---------------------------------&&&& #####
            
            three_stage_6_segment_1 = self.pos_three_stage[3830][1] - self.pos_three_stage[7][1]
            #print("three_stage_6_segment_1 length is :", three_stage_6_segment_1) 
            
            
            epsilon_61 = ((three_stage_6_segment_1 - 2.8257503509586996)/2.8257503509586996)*100
        
            r61 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_61 + 2711.890962292 )
          
                
            
                        
#########&&&&----------------  part6  segments  ---------------------------------&&&& #####
            
            three_stage_6_segment_2 = self.pos_three_stage[4547][1] - self.pos_three_stage[3830][1]
            #print("three_stage_6_segment_2 length is :", three_stage_6_segment_2) 
            
            
            epsilon_62 = ((three_stage_6_segment_2 - 2.844501495356411)/2.844501495356411)*100

            r62 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_62 + 2711.890962292 )
          
                    
            
             
                                     
#########&&&&----------------  part6  segments  ---------------------------------&&&& #####
            
            three_stage_6_segment_3 = self.pos_three_stage[5974][1] - self.pos_three_stage[4547][1]
            #print("three_stage_6_segment_3 length is :", three_stage_6_segment_3) 
            
            
            epsilon_63 = ((three_stage_6_segment_3 - 2.872749328612798)/2.872749328612798)*100
        
            r63 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_63 + 2711.890962292 )
          
                    
            
            
                                                 
#########&&&&----------------  part6  segments  ---------------------------------&&&& #####
            
            three_stage_6_segment_4 = self.pos_three_stage[2297][1] - self.pos_three_stage[5974][1]
            #print("three_stage_6_segment_4 length is :", three_stage_6_segment_4) 
            
            
            epsilon_64 = ((three_stage_6_segment_4 - 2.8584995269770985)/2.8584995269770985)*100
            
            r64 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_64 + 2711.890962292 )
          
                    
            
            
            #########&&&&----------------  part6  segments  ---------------------------------&&&& #####
            
            three_stage_6_segment_5 = self.pos_three_stage[1971][1] - self.pos_three_stage[2297][1]
            #print("three_stage_6_segment_5 length is :", three_stage_6_segment_5) 
            
            
            epsilon_65 = ((three_stage_6_segment_5 - 2.9345054626449922)/2.9345054626449922)*100
            
            r65 = r_spiral_segment_1/2711.890962292 *( 40.9786262984012 * epsilon_65 + 2711.890962292 )
          
                    
                      
            r_6sum = r61 + r62 + r63 + r64 + r65 
            epsilon_6sum = epsilon_61 + epsilon_62 + epsilon_63 + epsilon_64 + epsilon_65
            #print(r_6sum) 
            
                               
                                   
#***********************  part7 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_7 = self.pos_three_stage[1816][1] - self.pos_three_stage[1716][1]
            #print("three_stage_7 length is :", three_stage_7)
            
            
#########&&&&----------------  part7  segments  ---------------------------------&&&& #####
            
            three_stage_7_segment_1 = self.pos_three_stage[5152][1] - self.pos_three_stage[1716][1]
            #print("three_stage_7_segment_1 length is :", three_stage_7_segment_1) 
            
            
            epsilon_71 = ((three_stage_7_segment_1 - 1.834247589110646)/1.834247589110646)*100
        
            r71 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_71 + 2711.890962292 )
          
                    
            
            
            #########&&&&----------------  part7  segments  ---------------------------------&&&& #####
            
            three_stage_7_segment_2 = self.pos_three_stage[3254][1] - self.pos_three_stage[5152][1]
            #print("three_stage_7_segment_2 length is :", three_stage_7_segment_2) 
            
            
            epsilon_72 = ((three_stage_7_segment_2 - 1.8904991149904191)/1.8904991149904191)*100
    
            r72 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_72 + 2711.890962292 )
          
                    
            
            
        #########&&&&----------------  part7  segments  ---------------------------------&&&& #####
            
            three_stage_7_segment_3 = self.pos_three_stage[3240][1] - self.pos_three_stage[3252][1]
            #print("three_stage_7_segment_3 length is :", three_stage_7_segment_3) 
            
            
            epsilon_73 = ((three_stage_7_segment_3 - 1.8107490539550781)/1.8107490539550781)*100
        
            r73 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_73 + 2711.890962292 )
          
                    
            
            
    #########&&&&----------------  part7  segments  ---------------------------------&&&& #####
            
            three_stage_7_segment_4 = self.pos_three_stage[2360][1] - self.pos_three_stage[3240][1]
            #print("three_stage_7_segment_4 length is :", three_stage_7_segment_4) 
            
            
            epsilon_74 = ((three_stage_7_segment_4 - 1.8985023498538567)/1.8985023498538567)*100
    
            r74 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_74 + 2711.890962292 )
          
                    
            
                        
    #########&&&&----------------  part7  segments  ---------------------------------&&&& #####
            
            three_stage_7_segment_5 = self.pos_three_stage[1816][1] - self.pos_three_stage[2360][1]
            #print("three_stage_7_segment_5 length is :", three_stage_7_segment_5) 
            
            
            epsilon_75 = ((three_stage_7_segment_5 - 1.8819961547799977)/1.8819961547799977)*100
        
            r75 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_75 + 2711.890962292 )
          
                    
            
                      
            r_7sum = r71 + r72 + r73 + r74 + r75 
            epsilon_7sum = epsilon_71 + epsilon_72 + epsilon_73 + epsilon_74 + epsilon_75
      
            #print(r_7sum) 
            
                                             
#***********************  part8 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_8 = self.pos_three_stage[1811][1] - self.pos_three_stage[1792][1]
            #print("three_stage_8 length is :", three_stage_8)
            
            
##########&&&&----------------  part8  segments  ---------------------------------&&&& #####
            
            three_stage_8_segment_1 = self.pos_three_stage[698][1] - self.pos_three_stage[1792][1]
            #print("three_stage_8_segment_1 length is :", three_stage_8_segment_1) 
            
            
            epsilon_81 = ((three_stage_8_segment_1 - 0.899002075196023)/0.899002075196023)*100
        
            r81 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_81 + 2711.890962292 )
          
            
                        
##########&&&&----------------  part8  segments  ---------------------------------&&&& #####
            
            three_stage_8_segment_2 = self.pos_three_stage[1790][1] - self.pos_three_stage[698][1]
            #print("three_stage_8_segment_2 length is :", three_stage_8_segment_2) 
            
            
            epsilon_82 = ((three_stage_8_segment_2 - 0.8700027465820028)/0.8700027465820028)*100

            r82 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_82 + 2711.890962292 )
          
            
            
                                    
##########&&&&----------------  part8  segments  ---------------------------------&&&& #####
            
            three_stage_8_segment_3 = self.pos_three_stage[2605][1] - self.pos_three_stage[1790][1]
            #print("three_stage_8_segment_3 length is :", three_stage_8_segment_3) 
            
            
            epsilon_83 = ((three_stage_8_segment_3 - 0.9869956970214844)/0.9869956970214844)*100
        
            r83 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_83 + 2711.890962292 )
          
            
            
                                                
##########&&&&----------------  part8  segments  ---------------------------------&&&& #####
            
            three_stage_8_segment_4 = self.pos_three_stage[1122][1] - self.pos_three_stage[2605][1]
            #print("three_stage_8_segment_4 length is :", three_stage_8_segment_4) 
            
            
            epsilon_84 = ((three_stage_8_segment_4 - 0.9415016174287985)/0.9415016174287985)*100
        
            r84 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_84 + 2711.890962292 )
          
            
            
    ##########&&&&----------------  part8  segments  ---------------------------------&&&& #####
            
            three_stage_8_segment_5 = self.pos_three_stage[1811][1] - self.pos_three_stage[1122][1]
            #print("three_stage_8_segment_5 length is :", three_stage_8_segment_5) 
            
            
            epsilon_85 = ((three_stage_8_segment_5 - 0.9444961547826978)/0.9444961547826978)*100
    
            r85 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_85 + 2711.890962292 )
          
                      
            r_8sum = r81 + r82 + r83 + r84 + r85 
            epsilon_8sum = epsilon_81 + epsilon_82 + epsilon_83 + epsilon_84 + epsilon_85
            #print(r_8sum)
            
            
        
                                                       
#***********************  part9 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_9 = self.pos_three_stage[1732][0] - self.pos_three_stage[1776][0]
            #print("three_stage_9 length is :", three_stage_9)
            
            
###########&&&&----------------  part9  segments  ---------------------------------&&&& #####
            
            three_stage_9_segment_1 = self.pos_three_stage[5501][0] - self.pos_three_stage[1776][0]
            ##print("three_stage_9_segment_1 length is :", three_stage_9_segment_1) 
            
            
            epsilon_91 = ((three_stage_9_segment_1 - 1.93424892425503)/1.93424892425503)*100
        
            
            r_91 = (r_spiral_segment_2/7161) * (7161 - 647 *  epsilon_91 + 25.2 *  epsilon_91**2 - 0.327 *  epsilon_91**3)
            r91 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_91 + 2711.890962292 )
          
              
            
###########&&&&----------------  part9  segments  ---------------------------------&&&& #####
            
            three_stage_9_segment_2 = self.pos_three_stage[3503][0] - self.pos_three_stage[5501][0]
            #print("three_stage_9_segment_2 length is :", three_stage_9_segment_2) 
            
            
            epsilon_92 = ((three_stage_9_segment_2 -  1.9690008163454706)/ 1.9690008163454706)*100
        
            r92 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_92 + 2711.890962292 )
          
              
            #print(" r_92 is :", r_92)
            
    ###########&&&&----------------  part9  segments  ---------------------------------&&&& #####
            
            three_stage_9_segment_3 = self.pos_three_stage[1768][0] - self.pos_three_stage[3503][0]
            #print("three_stage_9_segment_3 length is :", three_stage_9_segment_3) 
            
            
            epsilon_93 = ((three_stage_9_segment_3 - 1.863748550414499)/ 1.863748550414499)*100
    
            r93 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_93 + 2711.890962292 )
            #print(" r_93 is :", r_93)
                      
        ###########&&&&----------------  part9  segments  ---------------------------------&&&& #####
            
            three_stage_9_segment_4 = self.pos_three_stage[5548][0] - self.pos_three_stage[5564][0]
            #print("three_stage_9_segment_4 length is :", three_stage_9_segment_4) 
            
            
            epsilon_94 = ((three_stage_9_segment_4 -  1.9492521286012447)/ 1.9492521286012447)*100
            r94 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_94 + 2711.890962292 )
            
            #print(" r_94 is :", r_94)
            
    ###########&&&&----------------  part9  segments  ---------------------------------&&&& #####
            
            three_stage_9_segment_5 = self.pos_three_stage[1732][0] - self.pos_three_stage[5548][0]
            #print("three_stage_9_segment_5 length is :", three_stage_9_segment_5) 
            
            
            epsilon_95 = ((three_stage_9_segment_5 - 1.9094982147215163)/ 1.9094982147215163)*100

            r95 = r_spiral_segment_2/2711.890962292 *( 40.9786262984012 * epsilon_95 + 2711.890962292 )
            
                                  
            r_9sum = r91 + r92 + r93 + r94 + r95 
            epsilon_9sum = epsilon_91 + epsilon_92 + epsilon_93 + epsilon_94 + epsilon_95
            #print(r_9sum)
            
            
            
                                                                
#***********************  part10 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_10 = self.pos_three_stage[1912][0] - self.pos_three_stage[1766][0]
            #print("three_stage_10 length is :", three_stage_10)
            
            
############&&&&----------------  part10  segments  ---------------------------------&&&& #####
            
            three_stage_10_segment_1 = self.pos_three_stage[4158][0] - self.pos_three_stage[1766][0]
            #print("three_stage_10_segment_1 length is :", three_stage_10_segment_1) 
            
            
            epsilon_101 = ((three_stage_10_segment_1 - 0.9567489624021803)/0.9567489624021803)*100
           
            r101 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_101 + 2711.890962292 )
            
            
            
                        
############&&&&----------------  part10  segments  ---------------------------------&&&& #####
            
            three_stage_10_segment_2 = self.pos_three_stage[1641][0] - self.pos_three_stage[4158][0]
            #print("three_stage_10_segment_2 length is :", three_stage_10_segment_2) 
            
            
            epsilon_102 = ((three_stage_10_segment_2 - 0.9907512664798261)/0.9907512664798261)*100
        
            r102 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_102 + 2711.890962292 )
            
            
            
                                    
############&&&&----------------  part10  segments  ---------------------------------&&&& #####
            
            three_stage_10_segment_3 = self.pos_three_stage[1184][0] - self.pos_three_stage[1641][0]
            #print("three_stage_10_segment_3 length is :", three_stage_10_segment_3) 
            
            
            epsilon_103 = ((three_stage_10_segment_3 - 0.8014984130860014)/0.8014984130860014)*100
            
            r103 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_103 + 2711.890962292 )
            
            
            
            ############&&&&----------------  part10  segments  ---------------------------------&&&& #####
            
            three_stage_10_segment_4 = self.pos_three_stage[4146][0] - self.pos_three_stage[1184][0]
            #print("three_stage_10_segment_4 length is :", three_stage_10_segment_4) 
            
            
            epsilon_104 = ((three_stage_10_segment_4 - 0.9905014038088495)/0.9905014038088495)*100
    
            r104 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_104 + 2711.890962292 )
            
            
            
        ############&&&&----------------  part10  segments  ---------------------------------&&&& #####
            
            three_stage_10_segment_5 = self.pos_three_stage[1912][0] - self.pos_three_stage[4146][0]
            #print("three_stage_10_segment_5 length is :", three_stage_10_segment_5) 
            
            
            epsilon_105 = ((three_stage_10_segment_5 - 0.9564990997311611)/0.9564990997311611)*100
            

            r105 = r_spiral_segment_3/2711.890962292 *( 40.9786262984012 * epsilon_105 + 2711.890962292 )
            
        
            r_10sum = r101 + r102 + r103 + r104 + r105 
            epsilon_10sum = epsilon_101 + epsilon_102 + epsilon_103 + epsilon_104 + epsilon_105
      
            #print(r_10sum)
            
            
      #######***********************  matrix length   ************************#


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
            matrix_ep = matrix_epsilon_1 + matrix_epsilon_2 + matrix_epsilon_3 + matrix_epsilon_4 + matrix_epsilon_5
            ####print(matrix_ep)


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

