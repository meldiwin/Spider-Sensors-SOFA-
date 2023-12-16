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
		self.pos_one_stage = kwargs['pos_one_stage']
		

            	
	def onAnimateBeginEvent(self,event):
            self.time = self.node.time.value
            self.pos_matrix= self.node.matrix.l_matrix.position.value
            self.pos_one_stage= self.node.matrix.one_stage.l_one_stage.position.value



            ramp_time = 5  # Time for each ramp (up and down)
            forces = np.piecewise(self.time , [self.time  < ramp_time, (ramp_time <= self.time ) & (self.time  <= 2*ramp_time), self.time  > 2*ramp_time],
                     [lambda t: 80000 * t, lambda t: 400000 - 80000 * (t - ramp_time), 0])

            self.node.matrix.FF.force.value = [0,forces,0]
            
      

               

#***********************  part1 in one stage "Spiral- X axis direction"  ************************#

              ##### full length part1 ####
            
            one_stage_1 = self.pos_one_stage[973][0] - self.pos_one_stage[978][0]
            print("one_stage_1 length is :", one_stage_1)
            
            
######&&&&----------------  part1  segments  ---------------------------------&&&& #####
            
            one_stage_1_segment_1 = self.pos_one_stage[395][0] - self.pos_one_stage[978][0]
            print("one_stage_1_segment_1 length is :", one_stage_1_segment_1) 
            
            
            epsilon_11 = ((one_stage_1_segment_1 - 2.9814987182618182)/2.9814987182618182)*100
            
            print(" epsilon_11 is :", epsilon_11) 
            
            r_11 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_11 + 25.2 *  epsilon_11**2 - 0.327 *  epsilon_11**3)
            
            print(" r_11 is :", r_11)
            
            
######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            one_stage_1_segment_2 = self.pos_one_stage[1904][0] - self.pos_one_stage[395][0]
            print("one_stage_1_segment_2 length is :", one_stage_1_segment_2) 
            
            
            epsilon_12 = ((one_stage_1_segment_2 - 2.736003875732301)/2.736003875732301)*100
            
            print(" epsilon_11 is :", epsilon_11) 
            
            r_12 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_12 + 25.2 *  epsilon_12**2 - 0.327 *  epsilon_12**3)
            
            print(" r_12 is :", r_12)
            
      
      
######&&&&----------------  part3  segments  ---------------------------------&&&& #####
            
            one_stage_1_segment_3 = self.pos_one_stage[32][0] - self.pos_one_stage[1904][0]
            print("one_stage_1_segment_3 length is :", one_stage_1_segment_3) 
            
            
            epsilon_13 = ((one_stage_1_segment_3 -  2.7544975280768824)/ 2.7544975280768824)*100
            
            print(" epsilon_13 is :", epsilon_13) 
            
            r_13 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_13 + 25.2 *  epsilon_13**2 - 0.327 *  epsilon_13**3)
            
            print(" r_13 is :", r_13)
            
      
            
######&&&&----------------  part4  segments  ---------------------------------&&&& #####
            
            one_stage_1_segment_4 = self.pos_one_stage[388][0] - self.pos_one_stage[32][0]
            print("one_stage_1_segment_4 length is :", one_stage_1_segment_4) 
            
            
            epsilon_14 = ((one_stage_1_segment_4 -  3.0424995422358023)/ 3.0424995422358023)*100
            
            print(" epsilon_14 is :", epsilon_14) 
            
            r_14 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_14 + 25.2 *  epsilon_14**2 - 0.327 *  epsilon_14**3)
            
            print(" r_14 is :", r_14)
            
      
      
            
######&&&&----------------  part5  segments  ---------------------------------&&&& #####
            
            one_stage_1_segment_5 = self.pos_one_stage[973][0] - self.pos_one_stage[388][0]
            print("one_stage_1_segment_5 length is :", one_stage_1_segment_5) 
            
            
            epsilon_15 = ((one_stage_1_segment_5 -  2.9815025329592046)/ 2.9815025329592046)*100
            
            print(" epsilon_15 is :", epsilon_15) 
            
            r_15 = (r_spiral_segment/7161) * (7161 - 647 *  epsilon_15 + 25.2 *  epsilon_15**2 - 0.327 *  epsilon_15**3)
            
            print(" r_15 is :", r_15)
                  
                        
   ######&&&&&&&&&&&&  R_ part1_ Summations &&&&&&&&&&&& #####
            
            r_1sum = r_11 + r_12 + r_13 + r_14 + r_15 
      
            print(r_1sum)
      
      
      
      
      
      
#***********************  part2 in one stage "Radial- Y axis direction"  ************************#

              ##### full length part1 ####
            
            one_stage_2 = self.pos_one_stage[233][0] - self.pos_one_stage[193][0]
            print("one_stage_2 length is :", one_stage_2)
            
            
#######&&&&----------------  part2  segments  ---------------------------------&&&& #####
            
            one_stage_2_segment_1 = self.pos_one_stage[2708][0] - self.pos_one_stage[193][0]
            print("one_stage_2_segment_1 length is :", one_stage_2_segment_1) 
            
            
            epsilon_21 = ((one_stage_2_segment_1 - 5.826808972675728)/5.826808972675728)*100
            
            #print(" epsilon_11 is :", epsilon_11) 
            
            r_21 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_21 + 25.2 *  epsilon_21**2 - 0.327 *  epsilon_21**3)
            
            print(" r_21 is :", r_21)
            
      
                  
#######&&&&--------------------------------------------------------------------
            
            one_stage_2_segment_2 = self.pos_one_stage[1233][0] - self.pos_one_stage[2358][0]
            print("one_stage_2_segment_2 length is :", one_stage_2_segment_2) 
            
            
            epsilon_22 = ((one_stage_2_segment_2 - 6.771688360358702)/6.771688360358702)*100
            
            print(" epsilon_22 is :", epsilon_22) 
            
            r_22 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_22 + 25.2 *  epsilon_22**2 - 0.327 *  epsilon_22**3)
            
            print(" r_22 is :", r_22)
            
            
                              
#######&&&&--------------------------------------------------------------------
            
            one_stage_2_segment_3 = self.pos_one_stage[1975][0] - self.pos_one_stage[1233][0]
            print("one_stage_2_segment_3 length is :", one_stage_2_segment_3) 
            
            
            epsilon_23 = ((one_stage_2_segment_3 - 5.590626716613642)/5.590626716613642)*100
            
            print(" epsilon_23 is :", epsilon_23) 
            
            r_23 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_23 + 25.2 *  epsilon_23**2 - 0.327 *  epsilon_23**3)
            
            print(" r_23 is :", r_23)
            
      
                                   
#######&&&&--------------------------------------------------------------------
            
            one_stage_2_segment_4 = self.pos_one_stage[886][0] - self.pos_one_stage[1975][0]
            print("one_stage_2_segment_4 length is :", one_stage_2_segment_4) 
            
            
            epsilon_24 = ((one_stage_2_segment_4 - 6.287375450134128)/6.287375450134128)*100
            
            print(" epsilon_24 is :", epsilon_24) 
            
            r_24 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_24 + 25.2 *  epsilon_24**2 - 0.327 *  epsilon_24**3)
            
            print(" r_24 is :", r_24)
            
            
                                               
#######&&&&--------------------------------------------------------------------
            
            one_stage_2_segment_5 = self.pos_one_stage[233][0] - self.pos_one_stage[1932][0]
            print("one_stage_2_segment_5 length is :", one_stage_2_segment_5) 
            
            
            epsilon_25 = ((one_stage_2_segment_5 - 6.327729680449302)/6.327729680449302)*100
            
            print(" epsilon_25 is :", epsilon_25) 
            
            r_25 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_25 + 25.2 *  epsilon_25**2 - 0.327 *  epsilon_25**3)
            
            print(" r_25 is :", r_25)
             
      
#***********************  part3 in one stage "Radial- Y axis direction"  ************************#

              ##### full length part1 ####
            
            one_stage_3 = self.pos_one_stage[207][1] - self.pos_one_stage[245][1]
            print("one_stage_3 length is :", one_stage_3)
            
            
#######&&&&----------------  part3  segments  ------------------------------&&&& #####
            
            one_stage_3_segment_1 = self.pos_one_stage[181][1] - self.pos_one_stage[245][1]
            print("one_stage_3_segment_1 length is :", one_stage_3_segment_1) 
            
            
            epsilon_31 = ((one_stage_3_segment_1 - 6.21900177002)/6.21900177002)*100
            
            print(" epsilon_31 is :", epsilon_31) 
            
            r_31 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_31 + 25.2 *  epsilon_31**2 - 0.327 *  epsilon_31**3)
            
            print(" r_31 is :", r_31)
            
            
            
     #### ------------------------------&&&& #####
            
            one_stage_3_segment_2 = self.pos_one_stage[2455][1] - self.pos_one_stage[2205][1]
            print("one_stage_3_segment_2 length is :", one_stage_3_segment_2) 
            
            
            epsilon_32 = ((one_stage_3_segment_2 - 6.044966940228008)/6.044966940228008)*100
            
            print(" epsilon_32 is :", epsilon_32) 
            
            r_32 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_32 + 25.2 *  epsilon_32**2 - 0.327 *  epsilon_32**3)
            
            print(" r_32 is :", r_32)
            
                   
            
                    
     #### ------------------------------&&&& #####
            
            one_stage_3_segment_3 = self.pos_one_stage[560][1] - self.pos_one_stage[2455][1]
            print("one_stage_3_segment_3 length is :", one_stage_3_segment_3) 
            
            
            epsilon_33 = ((one_stage_3_segment_3 - 6.0535844167047514)/6.0535844167047514)*100
            
            print(" epsilon_33 is :", epsilon_33) 
            
            r_33 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_33 + 25.2 *  epsilon_33**2 - 0.327 *  epsilon_33**3)
            
            print(" r_33 is :", r_33)    
            
            
            
    #### ------------------------------&&&& #####
            
            one_stage_3_segment_4 = self.pos_one_stage[593][1] - self.pos_one_stage[560][1]
            print("one_stage_3_segment_4 length is :", one_stage_3_segment_4) 
            
            
            epsilon_34 = ((one_stage_3_segment_4 - 6.067082722979379)/6.067082722979379)*100
            
            print(" epsilon_34 is :", epsilon_34) 
            
            r_34 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_34 + 25.2 *  epsilon_34**2 - 0.327 *  epsilon_34**3)
            
            print(" r_34 is :", r_34)    
            
            
            
                        
    #### ------------------------------&&&& #####
            
            one_stage_3_segment_5 = self.pos_one_stage[207][1] - self.pos_one_stage[593][1]
            print("one_stage_3_segment_5 length is :", one_stage_3_segment_5) 
            
            
            epsilon_35 = ((one_stage_3_segment_5 - 5.7672500610404995)/5.7672500610404995)*100
            
            print(" epsilon_35 is :", epsilon_35) 
            
            r_35 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_35 + 25.2 *  epsilon_35**2 - 0.327 *  epsilon_35**3)
            
            print(" r_35 is :", r_35)  
            
            
            
            
            
            
            
            
                  
#***********************  part4 in one stage "Radial perpendicular to force- Y axis direction"  ************************#

              ##### full length part1 ####
            
            one_stage_4 = self.pos_one_stage[207][1] - self.pos_one_stage[245][1]
            print("one_stage_4 length is :", one_stage_4)
            
            
#######&&&&----------- ------------------------------&&&& #####
            
            one_stage_4_segment_1 = self.pos_one_stage[2205][1] - self.pos_one_stage[245][1]
            print("one_stage_4_segment_1 length is :", one_stage_4_segment_1) 
            
            
            epsilon_41 = ((one_stage_4_segment_1 - 5.735115126623356)/5.735115126623356)*100
            
            print(" epsilon_41 is :", epsilon_41) 
            
            r_41 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_41 + 25.2 *  epsilon_41**2 - 0.327 *  epsilon_41**3)
            
            print(" r_41 is :", r_41)
            
            
            
                        
#######&&&&----------- ------------------------------&&&& #####
            
            one_stage_4_segment_2 = self.pos_one_stage[2446][1] - self.pos_one_stage[2205][1]
            print("one_stage_4_segment_2 length is :", one_stage_4_segment_2) 
            
            
            epsilon_42 = ((one_stage_4_segment_2 - 6.044966940228605)/ 6.044966940228605)*100
            
            print(" epsilon_42 is :", epsilon_42) 
            
            r_42 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_42 + 25.2 *  epsilon_42**2 - 0.327 *  epsilon_42**3)
            
            print(" r_41 is :", r_41)
            
            
            
                                    
#######&&&&----------- ------------------------------&&&& #####
            
            one_stage_4_segment_3 = self.pos_one_stage[925][1] - self.pos_one_stage[2446][1]
            print("one_stage_4_segment_3 length is :", one_stage_4_segment_3) 
            
            
            epsilon_43 = ((one_stage_4_segment_3 -  6.053584416703842)/ 6.053584416703842)*100
            
            print(" epsilon_43 is :", epsilon_43) 
            
            r_43 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_43 + 25.2 *  epsilon_43**2 - 0.327 *  epsilon_43**3)
            
            print(" r_43 is :", r_43)
    
    
                                        
#######&&&&----------- ------------------------------&&&& #####
            
            one_stage_4_segment_4 = self.pos_one_stage[629][1] - self.pos_one_stage[925][1]
            print("one_stage_4_segment_4 length is :", one_stage_4_segment_4) 
            
            
            epsilon_44 = ((one_stage_4_segment_4 - 6.0670827229797055)/ 6.0670827229797055)*100
            
            #print(" epsilon_44 is :", epsilon_44) 
            
            r_44 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_44 + 25.2 *  epsilon_44**2 - 0.327 *  epsilon_44**3)
            
            print(" r_44 is :", r_44)
          
          
          #######&&&&----------- ------------------------------&&&& #####
            
            one_stage_4_segment_5 = self.pos_one_stage[207][1] - self.pos_one_stage[629][1]
            print("one_stage_4_segment_5 length is :", one_stage_4_segment_5) 
            
            
            epsilon_45 = ((one_stage_4_segment_5 -  5.767250061040485)/ 5.767250061040485)*100
            
            ##print(" epsilon_45 is :", epsilon_45) 
            
            r_45 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_45 + 25.2 *  epsilon_45**2 - 0.327 *  epsilon_45**3)
            
            print(" r_45 is :", r_45)
          
          
          
          
          
                    
                  
#***********************  part5 in one stage "Spiral - Y axis direction"  ************************#

              ##### full length part1 ####
            
            one_stage_5 = self.pos_one_stage[][1] - self.pos_one_stage[][1]
            print("one_stage_5 length is :", one_stage_5)
            
            
########&&&&----------- ------------------------------&&&& #####
            
            #one_stage_4_segment_1 = self.pos_one_stage[2205][1] - self.pos_one_stage[245][1]
            #print("one_stage_4_segment_1 length is :", one_stage_4_segment_1) 
            
            
            #epsilon_41 = ((one_stage_4_segment_1 - 5.735115126623356)/5.735115126623356)*100
            
            #print(" epsilon_41 is :", epsilon_41) 
            
            #r_41 = (r_radial_segment/7161) * (7161 - 647 *  epsilon_41 + 25.2 *  epsilon_41**2 - 0.327 *  epsilon_41**3)
            
            #print(" r_41 is :", r_41)
            
            
            
            
            
      ######***********************  matrix strain   ************************#


            matrix_length = self.pos_matrix[1][1] - self.pos_matrix[6][1]
            print(" matrix_length is :",   matrix_length)
            

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
    # one_stage                           #
    ##########################################
    #  This add a new node in the scene. This node is appended to the matrix's node.
    one_stage = matrix.addChild('one_stage')
    one_stage.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.2, rayleighMass=0.2)
    one_stage.addObject('SparseLDLSolver')
    #  This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a pneumatic actuation it is a set of positions describing the spider wall.
    one_stage.addObject('MeshVTKLoader', name='loader', filename=path + '1_rect_trim_0.vtk', rotation=[0, 0, 0])
    one_stage.addObject('MeshTopology', src='@loader', name='topo')
    one_stage.addObject('MechanicalObject', name='l_one_stage')
    one_stage.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.4,
                        youngModulus=9e6)
    one_stage.addObject('UniformMass', totalMass=0.003)
    #one_stage.addObject('LinearSolverConstraintCorrection')


    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    #  between the spider wall (surfacic mesh) and the matrix (volumetric mesh) so that movements of the spider's DoFs will be mapped
    #  to the matrix and vice-versa;
    one_stage.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
    one_stage.addObject(SpiderController(node=rootNode, pos_matrix = rootNode.matrix.l_matrix.position.value,  pos_one_stage = rootNode.matrix.one_stage.l_one_stage.position.value))

    ##########################################
    # one_stage Visualization                          
    ##########################################
    one_stageVisu = one_stage.addChild('visu1')
    one_stageVisu.addObject('MeshSTLLoader', filename=path + "1_rect_trim_0.stl", name="loader")
    one_stageVisu.addObject('OglModel', src="@loader", color=[0.1, 0.1, 0.1, 0.9])
        
    one_stageVisu.addObject('TriangleCollisionModel')
    one_stageVisu.addObject('LineCollisionModel')
    one_stageVisu.addObject('PointCollisionModel')

    one_stageVisu.addObject('BarycentricMapping')


    return rootNode
