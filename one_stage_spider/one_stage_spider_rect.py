9# -*- coding: utf-8 -*-

import Sofa

from scipy.linalg import solve
import numpy as np


import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

################################ Paramters ##################################
R_radial = 1602.5  #one 
R_spiral  = 1731  # one 



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
            
            thirteen = self.pos_three_stage[495][1] - self.pos_three_stage[1957][1]
            print("thirteen is :", thirteen)
            
            epsilon_13 = ((thirteen - 13.831996917724297)/13.831996917724297)*100
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r13 = R_radial/2711.890962292 *( 40.9786262984012 * epsilon_13 + 2711.890962292 )
            #print(r1)
            
            ############################
            fourteen = self.pos_three_stage[508][0] - self.pos_three_stage[1952][0]
            print("fourteen is :", fourteen)
            
            epsilon_14 = ((fourteen - 13.985000610351008)/13.985000610351008)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r14 = R_radial/2711.890962292 *( 40.9786262984012 * epsilon_14 + 2711.890962292 )
            #print(r1)
            
            
            ############################
            fifteen = self.pos_three_stage[343][1] - self.pos_three_stage[2][1]
            print("fifteen is :", fifteen)
            
            epsilon_15 = ((fifteen - 13.795997619627997)/13.795997619627997)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r15 = R_radial /2711.890962292 *( 40.9786262984012 * epsilon_15 + 2711.890962292 )
            #print(r1)
            
            
            ############################
            sixteen = self.pos_three_stage[1941][1] - self.pos_three_stage[456][1] 
            print("sixteen is :", sixteen)
            
            epsilon_16 = ((sixteen - 13.83149719237882)/13.83149719237882)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r16 = R_radial/2711.890962292 *( 40.9786262984012 * epsilon_16 + 2711.890962292 )
            #print(r1)
            
        
            ############################
            seventeen = self.pos_three_stage[1960][0] - self.pos_three_stage[469][0]
            print("seventeen is :", seventeen)
            
            epsilon_17 = ((seventeen - 13.985000610351996)/13.985000610351996)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r17 = R_radial/2711.890962292 *( 40.9786262984012 * epsilon_17 + 2711.890962292 )
            #print(r1)
            
            ############################
            eighteen = self.pos_three_stage[482][1] - self.pos_three_stage[1958][1]
            print("eighteen is :", eighteen)
            
            epsilon_18 = ((eighteen - 13.831996917725505)/13.831996917725505)*100
            #print(epsilon_1 * 100)
            
            ###print(" epsilon_11 is :", epsilon_11) 
            r18 = R_radial/2711.890962292 *( 40.9786262984012 * epsilon_18 + 2711.890962292 )
            #print(r1)
            
            
  
            
    ################################# Spirals ##############################################
            
               ######### thirteen ###########
            seven = self.pos_three_stage[26][1] - self.pos_three_stage[14][1]
            print("seven is :", seven)
            
            epsilon_7 = ((seven - 2.934501647948508 )/ 2.934501647948508)*100
    
            r7 = R_spiral/2711.890962292 *( 40.9786262984012 * epsilon_7 + 2711.890962292 )
         
         
            ######### thirteen ###########
            eight = self.pos_three_stage[13][1] - self.pos_three_stage[79][1]
            print("eight is :", eight)
            
            epsilon_8 = ((eight - 14.335998535155994 )/ 14.335998535155994)*100
    
            r8 = R_spiral/2711.890962292 *( 40.9786262984012 * epsilon_8 + 2711.890962292 )
         
         
            ######### thirteen ###########
            nine = self.pos_three_stage[78][0] - self.pos_three_stage[66][0]
            print("nine is :", nine)
            
            epsilon_9 = ((nine - 14.496002197266009 )/ 14.496002197266009)*100
    
            r9 =  R_spiral/2711.890962292 *( 40.9786262984012 * epsilon_9 + 2711.890962292 )
            
               
            ######### thirteen ###########
            ten = self.pos_three_stage[53][1] - self.pos_three_stage[65][1] 
            print("ten is :", ten)
            
            epsilon_10 = ((ten - 14.335998535156023 )/ 14.335998535156023)*100
    
            r10 = R_spiral/2711.890962292 *( 40.9786262984012 * epsilon_10 + 2711.890962292 )
         
            ######### thirteen ###########
            eleven = self.pos_three_stage[40][1] - self.pos_three_stage[52][1] 
            print("eleven is :", eleven)
            
            epsilon_11 = ((eleven - 14.335998535159973 )/ 14.335998535159973)*100
    
            r11 = R_spiral/2711.890962292 *( 40.9786262984012 * epsilon_11 + 2711.890962292 )
         
                  
            ######### thirteen ###########
            twelve = self.pos_three_stage[27][0] - self.pos_three_stage[39][0]
            print("twelve is :", twelve)
            
            epsilon_12 = ((twelve - 14.496002197266002)/14.496002197266002)*100
    
            r12 = R_spiral/2711.890962292 *( 40.9786262984012 * epsilon_12 + 2711.890962292 )
         
           ######************ Kirchoff equations ****########
            A = np.zeros((18,18))
            I = np.zeros((18,1))
            C = np.zeros((18,1))
            I[0] = 1
            
            ### nodes first stage 
            A[0][0]= 1;   A[0][6]= -1;     A[0][12] = -1;	A[0][11] = 1;
            A[1][1]= 1;   A[1][7]= -1;	   A[1][13] = -1;	A[1][6] = 1;
            A[2][2]= 1;   A[2][8]= -1;	   A[2][14] = -1;	A[2][7] = 1;
            A[3][3]= 1;   A[3][9]= -1;	   A[3][15] = -1;	A[3][8] = 1;
            A[4][4]= 1;   A[4][10]= -1;    A[4][16] = -1;	A[4][9] = 1;
            A[5][5]= 1;	  A[5][11]= -1;    A[5][17] = -1;	A[5][10] = 1;


            #### loop first stage 
            A[6][6]=r7 ;    A[6][13]=r14;   A[6][12] = r13;
            A[7][7]=r8 ;    A[7][13]=r14;   A[7][14] = r15;
            A[8][8]=r9 ;    A[8][14]=r15;   A[8][15] = r16;
            A[9][9]=r10;    A[9][15]=r16;   A[6][16] = r17;
            A[10][10]=r11;  A[10][16]=r17;  A[10][17] = r18;
            A[11][11]=r12 ; A[11][17]=r18;  A[11][12] = r13;


########### Boundary Conditions 1 and 4 main radial ############## 

            A[12][0] = 1 ;  C[12][0] = 1;  
            A[13][1] = 1 ;  C[13][0] = 0;
            A[14][2] = 1 ;  C[14][0] = 0;
            A[15][3] = 1 ;  C[15][0] = -1;
            A[16][4] = 1 ;  C[16][0] = 0;
            A[17][5] = 1 ;  C[17][0] = 0;

    ## Solve the matrix equation
            I = solve(A, C)
            

###### Access the solution######
#####  I[7] is I_6 ########

            V = r7 * I[6] + r8 * I[7] + r9 * I[8];
            R_t14 = V;
            print(R_t14)
#
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
    three_stage.addObject('MeshVTKLoader', name='loader', filename=path + '1_rect_trim_0.vtk', rotation=[0, 0, 0])
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
    three_stageVisu.addObject('MeshSTLLoader', filename=path + "1_rect_trim_0.stl", name="loader")
    three_stageVisu.addObject('OglModel', src="@loader", color=[0.1, 0.1, 0.1, 0.9])
        
    three_stageVisu.addObject('TriangleCollisionModel')
    three_stageVisu.addObject('LineCollisionModel')
    three_stageVisu.addObject('PointCollisionModel')

    three_stageVisu.addObject('BarycentricMapping')


    return rootNode

