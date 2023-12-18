9# -*- coding: utf-8 -*-

import Sofa

import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

################################ Paramters ##################################
R_radial = 3205  #one 
R_spiral  = 1731  # one 

r_radial_segment = 641
r_spiral_segment = 346.2


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
            
            r_11 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_11 + 25.2 *  epsilon_11**2 - 0.327 *  epsilon_11**3)
            #print(" r_11 is :", r_11)
            
            
            
            #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_2 = self.pos_three_stage[4140][1] - self.pos_three_stage[1130][1]
            #print("three_stage_1_segment_2 length is :", three_stage_1_segment_2) 
            
            
            epsilon_12 = ((three_stage_1_segment_2 - 5.7928733825667535)/5.7928733825667535)*100
            
       
            
            r_12 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_12 + 25.2 *  epsilon_12**2 - 0.327 *  epsilon_12**3)
            #print(" r_12 is :", r_12)
            
            
    

            #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_3 = self.pos_three_stage[1693][1] - self.pos_three_stage[4140][1]
            #print("three_stage_1_segment_3 length is :", three_stage_1_segment_3) 
            
            
            epsilon_13 = ((three_stage_1_segment_3 - 5.955623626707009)/5.955623626707009)*100
            
          
            
            r_13 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_13 + 25.2 *  epsilon_13**2 - 0.327 *  epsilon_13**3)
            #print(" r_13 is :", r_13)
            
            
            
            #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_4 = self.pos_three_stage[1054][1] - self.pos_three_stage[1693][1]
            #print("three_stage_1_segment_4 length is :", three_stage_1_segment_4) 
            
            
            epsilon_14 = ((three_stage_1_segment_4 - 6.043502807615141)/6.043502807615141)*100
            
          
            
            r_14 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_14 + 25.2 *  epsilon_14**2 - 0.327 *  epsilon_14**3)
            #print(" r_14 is :", r_14)
            
            
	
        #######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_1_segment_5 = self.pos_three_stage[60][1] - self.pos_three_stage[1054][1]
            #print("three_stage_1_segment_5 length is :", three_stage_1_segment_5) 
            
            
            epsilon_15 = ((three_stage_1_segment_5 - 5.813495635985063)/5.813495635985063)*100
            
          
            
            r_15 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_15 + 25.2 *  epsilon_15**2 - 0.327 *  epsilon_15**3)
            #print(" r_15 is :", r_15)
            
                                    
   ######&&&&&&&&&&&&  R_ part1_ Summations &&&&&&&&&&&& #####
            
            r_1sum = r_11 + r_12 + r_13 + r_14 + r_15 
      
            #print(r_1sum)
            
            
                             

#***********************  part2 in one stage "Radial parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_2 = self.pos_three_stage[72][1] - self.pos_three_stage[33][1]
            #print("three_stage_2 length is :", three_stage_2)
            
            
#######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_1 = self.pos_three_stage[983][1] - self.pos_three_stage[33][1]
            #print("three_stage_2_segment_1 length is :", three_stage_2_segment_1) 
            
            
            epsilon_21 = ((three_stage_2_segment_1 - 6.034500122069957)/6.034500122069957)*100
            
          
            
            r_21 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_21 + 25.2 *  epsilon_21**2 - 0.327 *  epsilon_21**3)
            #print(" r_21 is :", r_21)
            
                        
#######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_2 = self.pos_three_stage[2552][1] - self.pos_three_stage[983][1]
            #print("three_stage_2_segment_2 length is :", three_stage_2_segment_2) 
            
            
            epsilon_22 = ((three_stage_2_segment_2 - 5.980503082275561)/5.980503082275561)*100
            
          
            
            r_22 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_22 + 25.2 *  epsilon_22**2 - 0.327 *  epsilon_22**3)
            #print(" r_22 is :", r_22)
            
                                    
#######&&&&----------------  part2 segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_3 = self.pos_three_stage[4571][1] - self.pos_three_stage[2552][1]
            #print("three_stage_2_segment_3 length is :", three_stage_2_segment_3) 
            
            
            epsilon_23 = ((three_stage_2_segment_3 - 6.127998352047484)/6.127998352047484)*100
            
          
            
            r_23 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_23 + 25.2 *  epsilon_23**2 - 0.327 *  epsilon_23**3)
            #print(" r_23 is :", r_23)
            
            
            #######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_4 = self.pos_three_stage[890][1] - self.pos_three_stage[4571][1]
            #print("three_stage_2_segment_4 length is :", three_stage_2_segment_4) 
            
            
            epsilon_24 = ((three_stage_2_segment_4 - 6.052001953120012)/6.052001953120012)*100
            
          
            
            r_24 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_24 + 25.2 *  epsilon_24**2 - 0.327 *  epsilon_24**3)
            #print(" r_24 is :", r_24)
            
            
            #######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            three_stage_2_segment_5 = self.pos_three_stage[72][1] - self.pos_three_stage[890][1]
            #print("three_stage_2_segment_5 length is :", three_stage_2_segment_5) 
            
            
            epsilon_25 = ((three_stage_2_segment_5 - 5.8049964904799936)/5.8049964904799936)*100
            
          
            
            r_25 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_25 + 25.2 *  epsilon_25**2 - 0.327 *  epsilon_25**3)
            #print(" r_25 is :", r_25)
            
            r_2sum = r_21 + r_22 + r_23 + r_24 + r_25 
            
            #print(r_2sum)
                        
                        
            
            
            
            
#***********************  part3 in one stage "Radial parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_3 = self.pos_three_stage[850][0] - self.pos_three_stage[811][0]
            #print("three_stage_3 length is :", three_stage_3)
            
            
########&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_1 = self.pos_three_stage[1108][0] - self.pos_three_stage[811][0]
            #print("three_stage_3_segment_1 length is :", three_stage_3_segment_1) 
            
            
            epsilon_31 = ((three_stage_3_segment_1 - 5.961999893187119)/5.961999893187119)*100
            
          
            
            r_31 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_31 + 25.2 *  epsilon_31**2 - 0.327 *  epsilon_31**3)
            #print(" r_31 is :", r_31)
            
                        
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_2 = self.pos_three_stage[785][0] - self.pos_three_stage[1108][0]
            #print("three_stage_3_segment_2 length is :", three_stage_3_segment_2) 
            
            
            epsilon_32 = ((three_stage_3_segment_2 - 5.924001693726879)/5.961999893187119)*100
            
          
            
            r_32 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_32 + 25.2 *  epsilon_32**2 - 0.327 *  epsilon_32**3)
            
            #print(" r_32 is :", r_32)
            
            
                                 
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_3 = self.pos_three_stage[1240][0] - self.pos_three_stage[785][0]
            #print("three_stage_3_segment_3 length is :", three_stage_3_segment_3) 
            
            
            epsilon_33 = ((three_stage_3_segment_3 - 6.0789985656744605)/ 6.0789985656744605)*100
            
          
            
            r_33 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_33 + 25.2 *  epsilon_33**2 - 0.327 *  epsilon_33**3)
            #print(" r_33 is :", r_33)
            
            
                                             
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_4 = self.pos_three_stage[1065][0] - self.pos_three_stage[1240][0]
            #print("three_stage_3_segment_4 length is :", three_stage_3_segment_4) 
            
            
            epsilon_34 = ((three_stage_3_segment_4 - 6.069000244141243)/ 6.069000244141243)*100
            
          
            
            r_34 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_34 + 25.2 *  epsilon_34**2 - 0.327 *  epsilon_34**3)
            #print(" r_34 is :", r_34)
            
                                                         
########&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            three_stage_3_segment_5 = self.pos_three_stage[850][0] - self.pos_three_stage[1065][0]
            #print("three_stage_3_segment_5 length is :", three_stage_3_segment_5) 
            
            
            epsilon_35 = ((three_stage_3_segment_5 - 5.965999603270298)/ 5.965999603270298)*100
            
          
            
            r_35 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_35 + 25.2 *  epsilon_35**2 - 0.327 *  epsilon_35**3)
            #print(" r_35 is :", r_35)
            
            
            r_3sum = r_31 + r_32 + r_33 + r_34 + r_35 
            
            #print(r_3sum)
                        
                        
                        
                  
            
#***********************  part4 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_4 = self.pos_three_stage[33][0] - self.pos_three_stage[21][0]
            #print("three_stage_4 length is :", three_stage_4)
            
            
########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_1 = self.pos_three_stage[827][0] - self.pos_three_stage[21][0]
            #print("three_stage_4_segment_1 length is :", three_stage_4_segment_1) 
            
            
            epsilon_41 = ((three_stage_4_segment_1 - 2.9815025329589915)/2.9815025329589915)*100
            
          
            
            r_41 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_41 + 25.2 *  epsilon_41**2 - 0.327 *  epsilon_41**3)
            #print(" r_41 is :", r_41)
            
            
            
                        
########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_2 = self.pos_three_stage[5457][0] - self.pos_three_stage[827][0]
            #print("three_stage_4_segment_2 length is :", three_stage_4_segment_2) 
            
            
            epsilon_42 = ((three_stage_4_segment_2 - 2.7757492065430185)/2.7757492065430185)*100
            
          
            
            r_42 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_42 + 25.2 *  epsilon_42**2 - 0.327 *  epsilon_42**3)
            #print(" r_42 is :", r_42)
                        
            
            
                                    
########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_3 = self.pos_three_stage[3367][0] - self.pos_three_stage[3383][0]
            #print("three_stage_4_segment_3 length is :", three_stage_4_segment_3) 
            
            
            epsilon_43 = ((three_stage_4_segment_3 - 3.060247421264826)/3.060247421264826)*100
            
          
            
            r_43 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_43 + 25.2 *  epsilon_43**2 - 0.327 *  epsilon_43**3)
            #print(" r_43 is :", r_43)
                        
                        
                        
    ########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_4 = self.pos_three_stage[834][0] - self.pos_three_stage[3367][0]
            #print("three_stage_4_segment_4 length is :", three_stage_4_segment_4) 
            
            
            epsilon_44 = ((three_stage_4_segment_4 - 2.7360000610354547)/ 2.7360000610354547)*100
            
          
            
            r_44 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_44 + 25.2 *  epsilon_44**2 - 0.327 *  epsilon_44**3)
            #print(" r_44 is :", r_44)
            
            
                                    
    ########&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            three_stage_4_segment_5 = self.pos_three_stage[33][0] - self.pos_three_stage[834][0]
            #print("three_stage_4_segment_5 length is :", three_stage_4_segment_5) 
            
            
            epsilon_45 = ((three_stage_4_segment_5 - 2.981502532959013)/ 2.981502532959013)*100
            
          
            
            r_45 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_45 + 25.2 *  epsilon_45**2 - 0.327 *  epsilon_45**3)
            #print(" r_45 is :", r_45)
                        
                     
                                 
            r_4sum = r_41 + r_42 + r_43 + r_44 + r_45 
            
            #print(r_4sum)
                        
                        
                        
                        
                        
                 
#***********************  part5 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_5 = self.pos_three_stage[8][1] - self.pos_three_stage[20][1]
            #print("three_stage_5 length is :", three_stage_5)
            
            
########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_1 = self.pos_three_stage[2164][1] - self.pos_three_stage[20][1]
            #print("three_stage_5_segment_1 length is :", three_stage_5_segment_1) 
            
            
            epsilon_51 = ((three_stage_5_segment_1 - 2.934501647948508)/2.934501647948508)*100
            
          
            
            r_51 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_51 + 25.2 *  epsilon_51**2 - 0.327 *  epsilon_51**3)
            #print(" r_51 is :", r_51)
            
                
########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_2 = self.pos_three_stage[110][1] - self.pos_three_stage[2164][1]
            #print("three_stage_5_segment_2 length is :", three_stage_5_segment_2) 
            
            
            epsilon_52 = ((three_stage_5_segment_2 - 2.7385025024414773)/2.7385025024414773)*100
            
          
            
            r_52 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_52 + 25.2 *  epsilon_52**2 - 0.327 *  epsilon_52**3)
            #print(" r_52 is :", r_52)
                    
                        
                        
                        
    ########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_3 = self.pos_three_stage[2156][1] - self.pos_three_stage[110][1]
            #print("three_stage_5_segment_3 length is :", three_stage_5_segment_3) 
            
            
            epsilon_53 = ((three_stage_5_segment_3 - 2.8364982604980042)/2.8364982604980042)*100
            
          
            
            r_53 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_53 + 25.2 *  epsilon_53**2 - 0.327 *  epsilon_53**3)
            #print(" r_53 is :", r_53)
                    
                    
                    
                                          
                        
    ########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_4 = self.pos_three_stage[115][1] - self.pos_three_stage[2156][1]
            #print("three_stage_5_segment_4 length is :", three_stage_5_segment_4) 
            
            
            epsilon_54 = ((three_stage_5_segment_4 - 2.867496490478999)/2.867496490478999)*100
            
          
            
            r_54 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_54 + 25.2 *  epsilon_54**2 - 0.327 *  epsilon_54**3)
            #print(" r_54 is :", r_54)
                    
                    
                    
                                            
    ########&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            three_stage_5_segment_5 = self.pos_three_stage[8][1] - self.pos_three_stage[115][1]
            #print("three_stage_5_segment_5 length is :", three_stage_5_segment_5) 
            
            
            epsilon_55 = ((three_stage_5_segment_5 - 2.958999633783975)/2.958999633783975)*100
            
          
            
            r_55 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_55 + 25.2 *  epsilon_55**2 - 0.327 *  epsilon_55**3)
            ##print(" r_55 is :", r_55)
                    
                    
                                          
            r_5sum = r_51 + r_52 + r_53 + r_54 + r_55 
      
            #print(r_5sum)
                    
                    
                    
                    
                                   
#***********************  part6 in one stage "Spiral parallel 2 force "  ************************#

              ###### full length part1 ####
            
            three_stage_6 = self.pos_three_stage[1971][1] - self.pos_three_stage[7][1]
            print("three_stage_6 length is :", three_stage_6)
            
            
#########&&&&----------------  part6  segments  ---------------------------------&&&& #####
            
            three_stage_6_segment_1 = self.pos_three_stage[3830][1] - self.pos_three_stage[7][1]
            print("three_stage_6_segment_1 length is :", three_stage_6_segment_1) 
            
            
            epsilon_61 = ((three_stage_6_segment_1 - 2.8257503509586996)/2.8257503509586996)*100
            
          
            
            r_61 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_61 + 25.2 *  epsilon_61**2 - 0.327 *  epsilon_61**3)
            print(" r_61 is :", r_61)
            
            
                        
#########&&&&----------------  part6  segments  ---------------------------------&&&& #####
            
            three_stage_6_segment_2 = self.pos_three_stage[][1] - self.pos_three_stage[3830][1]
            print("three_stage_6_segment_2 length is :", three_stage_6_segment_2) 
            
            
            #epsilon_62 = ((three_stage_6_segment_2 - 2.8257503509586996)/2.8257503509586996)*100
            
          
            
            #r_62 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_62 + 25.2 *  epsilon_62**2 - 0.327 *  epsilon_62**3)
            #print(" r_62 is :", r_62)
            
                        
      ######***********************  matrix strain   ************************#


            matrix_length = self.pos_matrix[1][1] - self.pos_matrix[6][1]
            #print(" matrix_length is :",   matrix_length)
            

            matrix_epsilon = ((matrix_length - 99.999999999997)/99.999999999997) * 100
            #print(matrix_epsilon)
            

                

	
		
		


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

