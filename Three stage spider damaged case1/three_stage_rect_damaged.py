9# -*- coding: utf-8 -*-

import Sofa

import SofaRuntime
SofaRuntime.importPlugin("SofaComponentAll")

################################ Paramters ##################################
R_filament = 1665.318 # ohm
filament_l = 25.980766296382015 # mm
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
		self.pos_filament = kwargs['pos_filament']
		

            	
	def onAnimateBeginEvent(self,event):
            self.time = self.node.time.value
            self.pos_matrix= self.node.matrix.l_matrix.position.value
            self.pos_filament= self.node.matrix.filament.l_filament.position.value



            ramp_time = 5  # Time for each ramp (up and down)
            forces = np.piecewise(self.time , [self.time  < ramp_time, (ramp_time <= self.time ) & (self.time  <= 2*ramp_time), self.time  > 2*ramp_time],
                     [lambda t: 80000 * t, lambda t: 400000 - 80000 * (t - ramp_time), 0])

            self.node.matrix.FF.force.value = [0,forces,0]
            
            
            
          ###########################  length changes #########################
          
            filament_length = self.pos_filament[6][1] - self.pos_filament[5][1]
            self.file_filament = open(path + 'position_filament.txt', 'w') 
            #print(filament_length, file= self.file_filament)
            print("filament length is :", filament_length )
            
             ###########################  segments #########################
            
            filament_segment_1 = self.pos_filament[74][1] - self.pos_filament[5][1]
            self.file_filament = open(path + 'position_filament.txt', 'w') 
            #print(filament_length, file= self.file_filament)
            #print("filament_segment_1 is :",filament_segment_1 )
                        
            epislon_1= (filament_segment_1 -  5.196)/  5.196
            print("epislon_1 :", epislon_1)
            
           
            
            
                        
            filament_segment_2 = self.pos_filament[78][1] - self.pos_filament[74][1]
            self.file_filament = open(path + 'position_filament.txt', 'w') 
            #print(filament_length, file= self.file_filament)
            #print("filament_segment_2 is :",filament_segment_2 )
            epislon_2= (filament_segment_2 -  5.196)/  5.196
            print("epislon_2 :", epislon_2)
            
            
                        
            filament_segment_3 = self.pos_filament[82][1] - self.pos_filament[78][1]
            self.file_filament = open(path + 'position_filament.txt', 'w') 
            #print(filament_length, file= self.file_filament)
            #print("filament_segment_3 is :",filament_segment_3 )
            epislon_3= (filament_segment_3 -  5.196)/  5.196
            print("epislon_3 :", epislon_3)
            
            filament_segment_4 = self.pos_filament[86][1] - self.pos_filament[82][1]
            self.file_filament = open(path + 'position_filament.txt', 'w') 
            #print(filament_length, file= self.file_filament)
            #print("filament_segment_3 is :",filament_segment_4 )
            epislon_4= (filament_segment_4 -  5.196)/  5.196
            print("epislon_4 :", epislon_4)
            
            
            filament_segment_5 = self.pos_filament[6][1] - self.pos_filament[86][1]
            self.file_filament = open(path + 'position_filament.txt', 'w') 
            #print(filament_length, file= self.file_filament)
            #print("filament_segment_5 is :",filament_segment_5 )
            epislon_5= (filament_segment_5 -  5.196)/  5.196
            print("epislon_5 :", epislon_5)
            
            
            
            
            
            ############# DMA  find the parameters ########################

            
            matrix_length = self.pos_matrix[1][1] - self.pos_matrix[6][1]
            self.file_matrix = open(path + 'position_matrix.txt', 'w')  
            #print(matrix_length, file= self.file_matrix)
            print("matrix length is :", matrix_length )
            
            
        #print(self.pos_matrix, file= self.file_matrix)
        #print(self.pos_matrix, file= self.file_matrix)
                

        
            
            
    


	
		
		
		


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
    # filament                           #
    ##########################################
    #  This add a new node in the scene. This node is appended to the matrix's node.
    filament = matrix.addChild('filament')
    filament.addObject('EulerImplicitSolver', firstOrder=False, rayleighStiffness=0.2, rayleighMass=0.2)
    filament.addObject('SparseLDLSolver')
    #  This adds a MechanicalObject, a component holding the degree of freedom of our
    # mechanical modelling. In the case of a pneumatic actuation it is a set of positions describing the spider wall.
    filament.addObject('MeshVTKLoader', name='loader', filename=path + '3_rect_damaged.vtk', rotation=[0, 0, 0])
    filament.addObject('MeshTopology', src='@loader', name='topo')
    filament.addObject('MechanicalObject', name='l_filament')
    filament.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.4,
                        youngModulus=9e6)
    filament.addObject('UniformMass', totalMass=0.003)
    #filament.addObject('LinearSolverConstraintCorrection')


    # This adds a BarycentricMapping. A BarycentricMapping is a key element as it will add a bi-directional link
    #  between the spider wall (surfacic mesh) and the matrix (volumetric mesh) so that movements of the spider's DoFs will be mapped
    #  to the matrix and vice-versa;
    filament.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)
    filament.addObject(SpiderController(node=rootNode, pos_matrix = rootNode.matrix.l_matrix.position.value,  pos_filament = rootNode.matrix.filament.l_filament.position.value))

    ##########################################
    # filament Visualization                          
    ##########################################
    filamentVisu = filament.addChild('visu1')
    filamentVisu.addObject('MeshSTLLoader', filename=path + "3_rect_damaged.stl", name="loader")
    filamentVisu.addObject('OglModel', src="@loader", color=[0.1, 0.1, 0.1, 0.9])
        
    filamentVisu.addObject('TriangleCollisionModel')
    filamentVisu.addObject('LineCollisionModel')
    filamentVisu.addObject('PointCollisionModel')

    filamentVisu.addObject('BarycentricMapping')


    return rootNode

