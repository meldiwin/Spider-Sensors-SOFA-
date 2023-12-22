9# -*- coding: utf-8 -*-

import Sofa

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
            print("one is :", one)
            
            epsilon_1 = ((one - 4.411502838134496)/4.411502838134496)*100
            
            ###print(" epsilon_11 is :", epsilon_11) 
            #r1= r1_0/2711.890962292 *( 40.9786262984012 * eps + 2711.890962292 )
            
            #R_radial = 3205
            ##print(" r_11 is :", r_11)
            
            
            
             ###### two ####
            
            two = self.pos_three_stage[2613][1] - self.pos_three_stage[2347][1]
            print("two is :", two)
            
            epsilon_2 = ((two - 4.049495697021499)/4.049495697021499)*100
            
                        
             ###### three ####
            
            three = self.pos_three_stage[2643][1] - self.pos_three_stage[2615][1]
            print("three is :", three)
            
            epsilon_3 = ((three - 3.969497680661007)/3.969497680661007)*100
            
            
                                    
             ###### four ####
            
            four = self.pos_three_stage[2524][1] - self.pos_three_stage[2649][1]
            print("four is :", four)
            epsilon_4 = ((four - 3.969501495365023)/3.969501495365023)*100
            
            
                                                
             ###### five ####
            
            five = self.pos_three_stage[2388][1] - self.pos_three_stage[2521][1]
            print("five is :", five)
            epsilon_5 = ((four - 4.6425056457500204)/4.6425056457500204)*100
            
                                                            
             ###### six ####
            
            six = self.pos_three_stage[2259][1] - self.pos_three_stage[2385][1]
            print("six is :", six)
            
            epsilon_6 = ((six - 3.6990013122550494)/3.6990013122550494)*100
            
                                                                        
             ###### seven ####
            
            seven = self.pos_three_stage[2339][0] - self.pos_three_stage[2146][0]
            print("seven is :", seven)
            epsilon_7 = ((seven - 3.7379989624019956)/3.7379989624019956)*100
            
                                                                                    
             ###### eight ####
            
            eight = self.pos_three_stage[2590][0] - self.pos_three_stage[2354][0]
            print("eight is :", eight)
            
            epsilon_8 = ((eight - 4.694999694824986)/4.694999694824986)*100
            
                                                                                                
             ###### nine ####
            
            nine = self.pos_three_stage[2659][0] - self.pos_three_stage[2596][0]
            print("nine is :", nine)
            
            epsilon_9 = ((nine - 4.01600265503)/4.01600265503)*100
            
            
            ###### ten ####
            
            ten = self.pos_three_stage[2563][0] - self.pos_three_stage[2661][0]
            print("ten is :", ten)
            
            epsilon_10 = ((ten - 4.013999938963998)/4.013999938963998)*100
            
            ###### eleven ####
            
            eleven = self.pos_three_stage[2396][0] - self.pos_three_stage[2530][0]
            print("eleven is :", eleven)
            
            epsilon_11 = ((eleven - 4.093997955322003)/4.093997955322003)*100
            
            ###### twelve ####
            
            twelve = self.pos_three_stage[2232][0] - self.pos_three_stage[2392][0]
            print("twelve is :", twelve)
            
            epsilon_12 = ((twelve - 4.4599990844730115 )/ 4.4599990844730115)*100
            
         
            
    ################################# Spirals ##############################################
            
               ######### thirteen ###########
            thirteen = self.pos_three_stage[8][1] - self.pos_three_stage[20][1]
            print("thirteen is :", thirteen)
            
            epsilon_13 = ((thirteen - 14.335998535150964 )/ 14.335998535150964)*100
            
            
                ######### fourteen ###########
            fourteen = self.pos_three_stage[119][1] - self.pos_three_stage[104][1]
            print("fourteen is :", fourteen)
            
            epsilon_14 = ((fourteen - 9.315994262690026 )/ 9.315994262690026)*100
            
                 ######### fiveteen ###########
            fiveteen = self.pos_three_stage[295][1] - self.pos_three_stage[301][1]
            print("fiveteen is :", fiveteen)
            
            epsilon_15 = ((fiveteen - 4.641998291010978 )/ 4.641998291010978)*100
            
            
            ######### sixteen ###########
            sixteen = self.pos_three_stage[33][0] - self.pos_three_stage[21][0]
            print("sixteen is :", sixteen)
            epsilon_16 = ((sixteen - 14.496002197266009 )/ 14.496002197266009)*100
            
            
            
            ######### seventeen ###########
            seventeen = self.pos_three_stage[182][0] - self.pos_three_stage[196][0]
            print("seventeen is :", seventeen)
            epsilon_17 = ((seventeen -  9.78099822998 )/ 9.78099822998)*100
            
                        
            ######### eighteen ###########
            eighteen = self.pos_three_stage[353][0] - self.pos_three_stage[347][0]
            print("eighteen is :", eighteen)
            epsilon_18 = ((eighteen - 4.6959991455080115 )/ 4.6959991455080115)*100
            
                                    
            ######### nineteen ###########
            nineteen = self.pos_three_stage[73][1] - self.pos_three_stage[7][1]
            print("nineteen is :", nineteen)
            epsilon_19 = (( nineteen  - 14.33600616455 )/ 14.33600616455)*100
            
            ######### twenty ###########
            twenty = self.pos_three_stage[84][1] - self.pos_three_stage[96][1]
            print("twenty is :", twenty)
            epsilon_20 = (( twenty  - 9.31600189209 )/ 9.31600189209)*100
            
                        
            ######### twenty_1 ###########
            twenty_1 = self.pos_three_stage[276][1] - self.pos_three_stage[282][1]
            print("twenty_1 is :", twenty_1)
            
            epsilon_21 = ((twenty_1 - 4.641998291020002 )/ 4.641998291020002)*100
            
                         
            ######### twenty_2 ###########
            twenty_2 = self.pos_three_stage[60][0] - self.pos_three_stage[72][0]
            print("twenty_2 is :", twenty_2)
            epsilon_22 = ((twenty_2 - 14.496002197265994 )/ 14.496002197265994)*100
            
                                     
            ######### twenty_3 ###########
            twenty_3 = self.pos_three_stage[222][0] - self.pos_three_stage[208][0]
            print("twenty_3 is :", twenty_3)
            epsilon_23 = ((twenty_3 - 9.780998229980014)/ 9.780998229980014)*100
            
            
                                                 
            ######### twenty_4 ###########
            twenty_4 = self.pos_three_stage[315][0] - self.pos_three_stage[321][0]
            print("twenty_4 is :", twenty_4)
            epsilon_24 = ((twenty_4 - 4.695999145508004)/ 4.695999145508004)*100
            
            
            ######### twenty_5 ###########
            twenty_5 = self.pos_three_stage[118][1] - self.pos_three_stage[106][1]
            print("twenty_5 is :", twenty_5)
            epsilon_25 = ((twenty_5 - 13.726997375482995)/ 13.726997375482995)*100
            
            ######### twenty_6 ###########
            twenty_6 = self.pos_three_stage[294][1] - self.pos_three_stage[302][1]
            print("twenty_6 is :", twenty_6)
            epsilon_26 = ((twenty_6 - 8.691993713374018)/ 8.691993713374018)*100
            
            
            ######### twenty_7 ###########
            twenty_7 = self.pos_three_stage[379][1] - self.pos_three_stage[381][1]
            print("twenty_7 is :", twenty_7)
            epsilon_27 = ((twenty_7 - 3.9689941406199694)/ 3.9689941406199694)*100
            
            ######### twenty_8 ###########
            twenty_8 = self.pos_three_stage[83][1] - self.pos_three_stage[97][1]
            print("twenty_8 is :", twenty_8)
            epsilon_28 = ((twenty_8 - 13.727005004890032)/ 13.727005004890032)*100
            
            
            ######### twenty_9 ###########
            twenty_9 = self.pos_three_stage[275][1] - self.pos_three_stage[283][1]
            print("twenty_9 is :", twenty_9)
            epsilon_29 = ((twenty_9 - 8.692001342780017 )/ 8.692001342780017)*100
            
            ######### thirty ###########
            thirty = self.pos_three_stage[377][1] - self.pos_three_stage[371][1]
            print(" thirty is :",  thirty)
            
            epsilon_30 = ((thirty - 3.969001770019986 )/ 3.969001770019986)*100
            
            
            ######### radials  ###########
            
            ######### thirty_1 ###########
            thirty_1 = self.pos_three_stage[209][1] - self.pos_three_stage[208][1]
            print(" thirty_1 is :",  thirty_1) 
            epsilon_31 = ((thirty_1 - 4.411994934079999)/ 4.411994934079999)*100
            
                        
            ######### thirty_2 ###########
            thirty_2 = self.pos_three_stage[322][1] - self.pos_three_stage[321][1]
            print(" thirty_2 is :",  thirty_2) 
            epsilon_32 = ((thirty_2 - 4.049003601069984 )/4.049003601069984 )*100
            
            
            ######### thirty_3 ###########
            thirty_3 = self.pos_three_stage[363][1] - self.pos_three_stage[362][1]
            print(" thirty_3 is :",  thirty_3) 
            epsilon_33 = ((thirty_3 - 3.970001220709989)/3.970001220709989)*100
            
                        
            ######### thirty_4 ###########
            thirty_4 = self.pos_three_stage[395][1] - self.pos_three_stage[394][1]
            print(" thirty_4 is :",  thirty_4) 
            epsilon_34 = ((thirty_4 - 3.968994140620012)/3.968994140620012)*100
            
            
            ######### thirty_5 ###########
            thirty_5 = self.pos_three_stage[254][1] - self.pos_three_stage[273][1]
            print(" thirty_5 is :",  thirty_5) 
            epsilon_35 = ((thirty_5 - 4.642997741699006)/4.642997741699006)*100
            
            
                        
            ######### thirty_6 ###########
            thirty_6 = self.pos_three_stage[170][1] - self.pos_three_stage[169][1]
            print(" thirty_6 is :",  thirty_6) 
            epsilon_36 = ((thirty_6 - 3.697998046874986)/3.697998046874986)*100

        
            
          
   
      #######***********************  matrix strain   ************************#


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

