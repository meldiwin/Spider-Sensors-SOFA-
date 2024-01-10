9# -*- coding: utf-8 -*-

import Sofa

import SofaRuntime
import matplotlib.pyplot as plt

SofaRuntime.importPlugin("SofaComponentAll")

################################ Paramters ##################################
R_0 = 3205 # ohm  r segment =  385 ############################## R/ L = R_seg/ L_seg
r_segment = 641 #ohm
a= 2
b= 3
 
radial_l = 30 # mm
matrix_l = 100 ## mm



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
		self.pos_radial = kwargs['pos_radial']

        
		            

            	
	def onAnimateBeginEvent(self,event):
            self.time = self.node.time.value
            self.pos_matrix= self.node.matrix.l_matrix.position.value
            self.pos_radial= self.node.matrix.radial.l_radial.position.value



            ramp_time = 5  # Time for each ramp (up and down)
            forces = np.piecewise(self.time , [self.time  < ramp_time, (ramp_time <= self.time ) & (self.time  <= 2*ramp_time), self.time  > 2*ramp_time],
                     [lambda t: 80000 * t, lambda t: 400000 - 80000 * (t - ramp_time), 0])

            self.node.matrix.FF.force.value = [0,forces,0]
            
            

#***********************  part1 in radial  ************************#
            
            ##### full length part1 #####
        
            radial_1 = self.pos_radial[128][1] - self.pos_radial[80][1]
            #print("radial_1 length is :", radial_1)
            
            #####&&&&  part1  segments &&&& #####
            
            radial_1_segment_1 = self.pos_radial[254][1] - self.pos_radial[260][1]
            #print("radial_1_segment_1 length is :", radial_1_segment_1) 
            
            
            epsilon_11 = ((radial_1_segment_1 - 9.827201843257797)/9.827201843257797)*100
            
            #print(" epsilon_11 is :", epsilon_11) 
            
  ###---------------------------------------------------------------------------------------------         
            r_11 = (r_segment/7161) * (7161 - 647 *  epsilon_11 + 25.2 *  epsilon_11**2 - 0.327 *  epsilon_11**3)
            
            #print(" r_11 is :", r_11)
            
            
            radial_1_segment_2 = self.pos_radial[258][1] - self.pos_radial[254][1]
            #print("radial_1_segment_2 length is :", radial_1_segment_2) 
            
            epsilon_12 = 100*((radial_1_segment_2 - 9.53919982909764)/ 9.53919982909764)
            #print(" epsilon_12 is :", epsilon_12) 
###---------------------------------------------------------------------------------------------         
            r_12 = (r_segment/7161) * (7161 - 647 *  epsilon_12 + 25.2 *  epsilon_12**2 - 0.327 *  epsilon_12**3)
            
            #print(" r_12 is :", r_12)
            
           
            radial_1_segment_3 = self.pos_radial[138][1] - self.pos_radial[258][1]
            #print("radial_1_segment_3 length is :", radial_1_segment_3) 
            
            epsilon_13 = 100*((radial_1_segment_3 - 8.882396698001969)/ 8.882396698001969)
            #print(" epsilon_13 is :", epsilon_13) 
###---------------------------------------------------------------------------------------------         
            r_13 = (r_segment/7161) * (7161 - 647 *  epsilon_13 + 25.2 *  epsilon_13**2 - 0.327 *  epsilon_13**3)
            
            #print(" r_13 is :", r_13)
            
            
            
            radial_1_segment_4 = self.pos_radial[437][1] - self.pos_radial[206][1]
            #print("radial_1_segment_4 length is :", radial_1_segment_4) 
            
            epsilon_14 = 100*((radial_1_segment_4 - 9.827201843258024)/ 9.827201843258024)
            #print(" epsilon_14 is :", epsilon_14) 
###---------------------------------------------------------------------------------------------         
            r_14 = (r_segment/7161) * (7161 - 647 *  epsilon_14 + 25.2 *  epsilon_14**2 - 0.327 *  epsilon_14**3)
            
            #print(" r_14 is :", r_14)
            
            
            
            radial_1_segment_5 = self.pos_radial[128][1] - self.pos_radial[437][1]
            #print("radial_1_segment_5 length is :", radial_1_segment_5) 
            
            epsilon_15 = 100*((radial_1_segment_5 - 9.539199829101392)/9.539199829101392 )
            #print(" epsilon_15 is :", epsilon_15) 
                 
###---------------------------------------------------------------------------------------------         
            r_15 = (r_segment/7161) * (7161 - 647 *  epsilon_15 + 25.2 *  epsilon_15**2 - 0.327 *  epsilon_15**3)
            #print(" r_15 is :", r_15)
            
            
            
            
            #####&&&&&&&&&&&&  R_ part1_ Summations &&&&&&&&&&&& #####
            
            
            r_1sum = r_11+ r_12 + r_13 + r_14 + r_15
            #print(r_1sum)
           
           
            
#***********************  part2 in radial  ************************#


            ##### full length part2 ######
            
            radial_2 = self.pos_radial[214][1] - self.pos_radial[260][1]
            #print("radial_2 length is :", radial_2)
            
            
            #####&&&&  part2  segments &&&& #####
            
            radial_2_segment_1 = self.pos_radial[180][1] - self.pos_radial[80][1]
            #print("radial_2_segment_1 length is :", radial_2_segment_1) 
            
            epsilon_21 = 100*((radial_2_segment_1 - 9.82720184326186)/ 9.82720184326186)
            #print(" epsilon_21 is :", epsilon_21) 
            
###---------------------------------------------------------------------------------------------         
            r_21 = (r_segment/7161) * (7161 - 647 *  epsilon_21 + 25.2 *  epsilon_21**2 - 0.327 *  epsilon_21**3)
            #print(" r_21 is :", r_21)
            

            
            
            radial_2_segment_2 = self.pos_radial[184][1] - self.pos_radial[180][1]
            #print("radial_2_segment_2 length is :", radial_2_segment_2) 
            
            epsilon_22 = 100*((radial_2_segment_2 -9.53919982910179 )/ 9.53919982910179)
            #print(" epsilon_22 is :", epsilon_22) 
            
###---------------------------------------------------------------------------------------------         
            r_22 = (r_segment/7161) * (7161 - 647 *  epsilon_22 + 25.2 *  epsilon_22**2 - 0.327 *  epsilon_22**3)
            #print(" r_22 is :", r_22)
            
            
            
            radial_2_segment_3 = self.pos_radial[224][1] - self.pos_radial[184][1]
            #print("radial_2_segment_3 length is :", radial_2_segment_3) 
            
            epsilon_23 = 100*((radial_2_segment_3 - 9.01442972818657 )/9.01442972818657 )
            #print(" epsilon_23 is :", epsilon_23) 
            
###---------------------------------------------------------------------------------------------         
            r_23 = (r_segment/7161) * (7161 - 647 *  epsilon_23 + 25.2 *  epsilon_23**2 - 0.327 *  epsilon_23**3)
            #print(" r_23 is :", r_23)
            
            
            
            radial_2_segment_4 = self.pos_radial[239][1] - self.pos_radial[120][1]
            #print("radial_2_segment_4 length is :", radial_2_segment_4)
            
            epsilon_24 = 100*((radial_2_segment_4 - 9.539199829096844 )/9.539199829096844)
            #print(" epsilon_24 is :", epsilon_24) 
            
###---------------------------------------------------------------------------------------------         
            r_24 = (r_segment/7161) * (7161 - 647 *  epsilon_24 + 25.2 *  epsilon_24**2 - 0.327 *  epsilon_24**3)
            
            #print(" r_24 is :", r_24)
            
            
            radial_2_segment_5 = self.pos_radial[214][1] - self.pos_radial[239][1]
            #print("radial_2_segment_5 length is :", radial_2_segment_5) 
            
            epsilon_25 = 100*((radial_2_segment_5 - 9.827201843264618 )/9.827201843264618)
            #print(" epsilon_25 is :", epsilon_25) 
            
###---------------------------------------------------------------------------------------------         
            r_25 = (r_segment/7161) * (7161 - 647 *  epsilon_25 + 25.2 *  epsilon_25**2 - 0.327 *  epsilon_25**3)
            
            #print(" r_25 is :", r_25)
            
            
            
             #####&&&&&&&&&&&&  R_ part2_ Summations &&&&&&&&&&&& #####
            
            r_2sum = r_21+ r_22 + r_23 + r_24 + r_25
            #print(r_2sum)
            
            
    
#***********************  part3 in radial  ************************#
            
            ##### full length part3 ######
            
            radial_3 = self.pos_radial[158][0] - self.pos_radial[250][0]
            #print("radial_3 length is :", radial_3)
            
            
            ######&&&&  part3  segments &&&& #####
            
            radial_3_segment_1 = self.pos_radial[247][0] - self.pos_radial[111][0]
            #print("radial_3_segment_1 length is :", radial_3_segment_1) 
            
            epsilon_31 = 100*((radial_3_segment_1 - 8.884500503539893 )/8.884500503539893)
            #print(" epsilon_31 is :", epsilon_31) 

###---------------------------------------------------------------------------------------------         
            r_31 = (r_segment/7161) * (7161 - 647 *  epsilon_31 + 25.2 *  epsilon_31**2 - 0.327 *  epsilon_31**3)
            #print(" r_31 is :", r_31)
            
            
            
            radial_3_segment_2 = self.pos_radial[503][0] - self.pos_radial[247][0]
            #print("radial_3_segment_2 length is :", radial_3_segment_2) 
            epsilon_32 = 100*((radial_3_segment_2 -  8.884500503539854 )/  8.884500503539854)
            #print(" epsilon_32 is :", epsilon_32) 
            
###---------------------------------------------------------------------------------------------         
            r_32 = (r_segment/7161) * (7161 - 647 *  epsilon_32 + 25.2 *  epsilon_32**2 - 0.327 *  epsilon_32**3)
            #print(" r_32 is :", r_32)
            
           
            
            radial_3_segment_3 = self.pos_radial[295][0] - self.pos_radial[104][0]
            #print("radial_3_segment_3 length is :", radial_3_segment_3) 
            epsilon_33 = 100*((radial_3_segment_3 - 8.469873905182467  )/ 8.469873905182467 )
            #print(" epsilon_33 is :", epsilon_33) 
            
###---------------------------------------------------------------------------------------------         
            r_33 = (r_segment/7161) * (7161 - 647 *  epsilon_33 + 25.2 *  epsilon_33**2 - 0.327 *  epsilon_33**3)
            #print(" r_33 is :", r_33)
            
            
            
            radial_3_segment_4 = self.pos_radial[201][0] - self.pos_radial[295][0]
            #print("radial_3_segment_4 length is :", radial_3_segment_4) 
            epsilon_34 = 100*((radial_3_segment_4 -  8.914124965667632 )/ 8.914124965667632 )
            #print(" epsilon_34 is :", epsilon_34) 

###---------------------------------------------------------------------------------------------         
            r_34 = (r_segment/7161) * (7161 - 647 * epsilon_34 + 25.2 * epsilon_34**2 - 0.327 *  epsilon_34**3)
            #print(" r_34 is :", r_34)
            
            
           
            
            radial_3_segment_5 = self.pos_radial[63][0] - self.pos_radial[66][0]
            #print("radial_3_segment_5 length is :", radial_3_segment_5) 
            
            epsilon_35 = 100*((radial_3_segment_5 - 8.91412496566764  )/ 8.91412496566764 )
            #print(" epsilon_35 is :", epsilon_35)

###---------------------------------------------------------------------------------------------         
            r_35 = (r_segment/7161) * (7161 - 647 *  epsilon_35 + 25.2 *  epsilon_35**2 - 0.327 *  epsilon_35**3)
            #print(" r_35 is :", r_35)
            
            
             #####&&&&&&&&&&&&  R_ part3_ Summations &&&&&&&&&&&& #####
                        
            r_3sum = r_31+ r_32 + r_33 + r_34 + r_35
            print(r_3sum)
            
    
            
    
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
    # radial                           #
    ##########################################
    #  This add a new node in the scene. This node is appended to the matrix's node.
    radial = matrix.addChild('radial')
    radial.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.2, rayleighMass=0.2)
    radial.addObject('SparseLDLSolver')
    #  This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a pneumatic actuation it is a set of positions describing the spider wall.
    radial.addObject('MeshVTKLoader', name='loader', filename=path + 'radial_rect_0.vtk', rotation=[0, 0, 0])
    radial.addObject('MeshTopology', src='@loader', name='topo')
    radial.addObject('MechanicalObject', name='l_radial')
    radial.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.4,
                        youngModulus=9e6)
    radial.addObject('UniformMass', totalMass=0.003)
    #radial.addObject('LinearSolverConstraintCorrection')


    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    #  between the spider wall (surfacic mesh) and the matrix (volumetric mesh) so that movements of the spider's DoFs will be mapped
    #  to the matrix and vice-versa;
    radial.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
    radial.addObject(SpiderController(node=rootNode, pos_matrix = rootNode.matrix.l_matrix.position.value,  pos_radial = rootNode.matrix.radial.l_radial.position.value))

    ##########################################
    # radial Visualization                          
    ##########################################
    radialVisu = radial.addChild('visu1')
    radialVisu.addObject('MeshSTLLoader', filename=path + "radial_rect_0.stl", name="loader")
    radialVisu.addObject('OglModel', src="@loader", color=[0.1, 0.1, 0.1, 0.9])
        
    radialVisu.addObject('TriangleCollisionModel')
    radialVisu.addObject('LineCollisionModel')
    radialVisu.addObject('PointCollisionModel')

    radialVisu.addObject('BarycentricMapping')
     
    
    
    return rootNode
    
