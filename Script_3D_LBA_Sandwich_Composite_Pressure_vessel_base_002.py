#python #abaqus #abaqustutorial #hnrwagner 


from abaqus import *
from abaqusConstants import *
import regionToolset
import __main__
import section
import regionToolset
import part
import material
import assembly
import step
import interaction
import load
import mesh
import job
import sketch
import visualization
import xyPlot
import connectorBehavior
import odbAccess
from operator import add
import numpy as np



# functions



def Create_Part_3D_Cylinder(radius,length,thickness,part,model):
    s1 = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=200.0)
    g, v, d, c = s1.geometry, s1.vertices, s1.dimensions, s1.constraints
    s1.setPrimaryObject(option=STANDALONE)
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius, 0.0))
    s1.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius-thickness, 0.0))
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseSolidExtrude(sketch=s1, depth=length)
    s1.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']
    
#----------------------------------------------------------------------------

def Create_Part_3D_Hemi_Sphere(radius,thickness,part,model):    
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=5000.0)
    g, v, d, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=STANDALONE)
    s.ConstructionLine(point1=(0.0, -2500.0), point2=(0.0, 2500.0))
    s.FixedConstraint(entity=g[2])
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius, 0.0), point2=(0.0, radius), direction=COUNTERCLOCKWISE)
    s.ArcByCenterEnds(center=(0.0, 0.0), point1=(radius-thickness, 0.0), point2=(0.0, radius-thickness), direction=COUNTERCLOCKWISE)
    s.Line(point1=(radius-thickness, 0.0), point2=(radius, 0.0))
    s.HorizontalConstraint(entity=g[5], addUndoState=False)
    s.Line(point1=(0.0, radius-thickness), point2=(0.0, radius))
    s.VerticalConstraint(entity=g[6], addUndoState=False)
    p = mdb.models[model].Part(name=part, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    p = mdb.models[model].parts[part]
    p.BaseSolidRevolve(sketch=s, angle=360.0, flipRevolveDirection=OFF)
    s.unsetPrimaryObject()
    p = mdb.models[model].parts[part]
    del mdb.models[model].sketches['__profile__']

#----------------------------------------------------------------------------

def Create_Datum_Plane_by_Principal(type_plane,part,model,offset_plane):
    p = mdb.models[model].parts[part]
    myPlane = p.DatumPlaneByPrincipalPlane(principalPlane=type_plane, offset=offset_plane)
    myID = myPlane.id
    return myID


def Create_Set_All_Cells(model,part,set_name):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    p.Set(cells=c, name=set_name)


def Create_Material_Data(model,material_name,e11,e22,e33,nu12,nu13,nu23,g12,g13,g23,lts,lcs,tts,tcs,lss,tss):
    mdb.models[model].Material(name=material_name)
    mdb.models[model].materials[material_name].Elastic(type=ENGINEERING_CONSTANTS, table=((e11,e22,e33,nu12,nu13,nu23,g12,g13,g23), ))
    mdb.models[model].materials[material_name].HashinDamageInitiation(table=((lts,lcs,tts,tcs,lss,tss), ))

def Create_Set_Face(x,y,z,model,part,set_name):
    face = ()
    p = mdb.models[model].parts[part]
    f = p.faces
    myFace = f.findAt((x,y,z),)
    face = face + (f[myFace.index:myFace.index+1], )
    p.Set(faces=face, name=set_name)
    return myFace

def Create_Set_Edge(x,y,z,model,part,set_name):
    edge = ()
    p = mdb.models[model].parts[part]
    e = p.edges
    myEdge = e.findAt((x,y,z),)
    edge = edge + (e[myEdge.index:myEdge.index+1], )
    f = p.Set(edges=edge, name=set_name)
    return myEdge

#-----------------------------------------------------------------------------

def Create_Set_Edge_Multi(x,y,z,x2,y2,z2,model,part,set_name):
    edge = ()
    p = mdb.models[model].parts[part]
    e = p.edges
    myEdge = e.findAt((x,y,z),)
    edge = edge + (e[myEdge.index:myEdge.index+1], )
    f = p.Set(edges=edge, name=set_name)
    myEdge = e.findAt((x2,y2,z2),)
    edge = edge + (e[myEdge.index:myEdge.index+1], )
    f = p.Set(edges=edge, name=set_name)
    return myEdge

#-----------------------------------------------------------------------------
def Create_Set_Vertice(x,y,z,model,part,set_name):
    vertice = ()
    p = mdb.models[model].parts[part]
    v = p.vertices
    myVertice = v.findAt((x,y,z),)
    vertice = vertice + (v[myVertice.index:myVertice.index+1], )
    p.Set(vertices=vertice, name=set_name)  
       

def Create_Set_Surface(x,y,z,model,part,set_name):
    face = ()
    p = mdb.models[model].parts[part]
    s = p.faces
    myFace = s.findAt((x,y,z),)
    face = face + (s[myFace.index:myFace.index+1], )
    p.Surface(side1Faces=face, name=set_name)
   

def Create_Assembly(model,part,instance_name):
    a = mdb.models[model].rootAssembly
    a.DatumCsysByDefault(CARTESIAN)
    p = mdb.models[model].parts[part]
    a.Instance(name=instance_name, part=p, dependent=ON)

#-------------------------------------------------------------

def Create_Reference_Point(x,y,z,model,setname):
    a = mdb.models[model].rootAssembly
    myRP = a.ReferencePoint(point=(x, y, z))
    r = a.referencePoints
    myRP_Position = r.findAt((x, y, z),)
    refPoints1=(myRP_Position, )
    a.Set(referencePoints=refPoints1, name=setname)
    return myRP,myRP_Position

def Create_Constraint_Equation(model,constraint_name,set_name,set_name_rp):
    mdb.models[model].Equation(name=constraint_name, terms=((1.0, set_name, 3), (-1.0, set_name_rp, 3)))


def Create_Boundary_Condition_by_Instance(model,instance_name,set_name,BC_name,step_name,u1_BC,u2_BC,u3_BC,ur1_BC,ur2_BC,ur3_BC):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].sets[set_name]
    mdb.models[model].DisplacementBC(name=BC_name, createStepName=step_name, region=region, u1=u1_BC, u2=u2_BC, u3=u3_BC, ur1=ur1_BC, ur2=ur2_BC, ur3=ur3_BC, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)  


def Create_Boundary_Condition_by_RP(model,RP_name,BC_name,step_name,u1_BC,u2_BC,u3_BC,ur1_BC,ur2_BC,ur3_BC):
    a = mdb.models[model].rootAssembly
    region = a.sets[RP_name]
    mdb.models[model].DisplacementBC(name=BC_name, createStepName=step_name, region=region, u1=u1_BC, u2=u2_BC, u3=u3_BC, ur1=ur1_BC, ur2=ur2_BC, ur3=ur3_BC, amplitude=UNSET, distributionType=UNIFORM, fieldName='', localCsys=None)  


def Create_Analysis_Step(model,step_name,pre_step_name,Initial_inc,Max_inc,Min_inc,Inc_Number,NL_ON_OFF):
    a = mdb.models[model].StaticStep(name=step_name, previous=pre_step_name, initialInc=Initial_inc, maxInc=Max_inc, minInc=Min_inc)
    a = mdb.models[model].steps[step_name].setValues(maxNumInc=Inc_Number)
    a = mdb.models[model].steps[step_name].setValues(nlgeom=NL_ON_OFF)
    a = mdb.models[model].steps[step_name].setValues(stabilizationMagnitude=1E-009, stabilizationMethod=DAMPING_FACTOR, continueDampingFactors=False, adaptiveDampingRatio=None)
    a = mdb.models[model].fieldOutputRequests['F-Output-1'].setValues(variables=('S', 'PE', 'PEEQ', 'PEMAG', 'LE', 'U', 'RF', 'CF', 'P', 'CSTRESS', 'CDISP', 'HSNFTCRT', 'HSNFCCRT', 'HSNMTCRT', 'HSNMCCRT'))

#-----------------------------------------------------------------------------

def Create_LBA_Step(model):
    mdb.models[model].BuckleStep(name='Step-1', previous='Initial', numEigen=10, vectors=18, maxIterations=3000)

#-----------------------------------------------------------------------------

def Create_Partion_by_Plane(model,part,id_plane):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    d = p.datums
    p.PartitionCellByDatumPlane(datumPlane=d[id_plane], cells=c)

#-----------------------------------------------------------------------------

def Create_Composite_Layup(model,part,set_name,composite_name,number,material,thickness,angle,surf,edge):
    layupOrientation = None
    p = mdb.models[model].parts[part]
    region1=p.sets[set_name]
    normalAxisRegion = p.surfaces[surf]
    primaryAxisRegion = p.sets[edge]
    compositeLayup = mdb.models[model].parts[part].CompositeLayup(name=composite_name, description='', elementType=CONTINUUM_SHELL, symmetric=False)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF)
    for i in range(0,number,1):
        compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material, thicknessType=SPECIFY_THICKNESS, thickness=thickness, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
        compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
   
#-----------------------------------------------------------------------------

def Create_Sandwich_Composite_Layup(model,part,set_name,composite_name,number,material,material_2,thickness,thickness_2,angle,surf,edge):
    layupOrientation = None
    p = mdb.models[model].parts[part]
    region1=p.sets[set_name]
    normalAxisRegion = p.surfaces[surf]
    primaryAxisRegion = p.sets[edge]
    compositeLayup = mdb.models[model].parts[part].CompositeLayup(name=composite_name, description='', elementType=CONTINUUM_SHELL, symmetric=False)
    compositeLayup.Section(preIntegrate=OFF, integrationRule=SIMPSON, poissonDefinition=DEFAULT, thicknessModulus=None, temperature=GRADIENT, useDensity=OFF)
    for i in range(0,number,1):
        if (angle[i] == 1):
            compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material_2, thicknessType=SPECIFY_THICKNESS, thickness=thickness_2, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
            compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
        else:
            compositeLayup.CompositePly(suppressed=False, plyName='Ply-'+str(i), region=region1, material=material, thicknessType=SPECIFY_THICKNESS, thickness=thickness, orientationType=SPECIFY_ORIENT, orientationValue=angle[i], additionalRotationType=ROTATION_NONE, additionalRotationField='', axis=AXIS_3, angle=0.0, numIntPoints=3)
            compositeLayup.ReferenceOrientation(orientationType=DISCRETE, localCsys=None, additionalRotationType=ROTATION_ANGLE, angle=90.0, additionalRotationField='', axis=AXIS_3, stackDirection=STACK_3, normalAxisDefinition=SURFACE, normalAxisRegion=normalAxisRegion, normalAxisDirection=AXIS_3, flipNormalDirection=False, primaryAxisDefinition=EDGE, primaryAxisRegion=primaryAxisRegion, primaryAxisDirection=AXIS_2, flipPrimaryDirection=False)
   
#----------------------------------------------------------------------------

def Create_Mesh(model,part,size):
    p = mdb.models[model].parts[part]
    elemType1 = mesh.ElemType(elemCode=SC8R, elemLibrary=STANDARD, secondOrderAccuracy=OFF, hourglassControl=DEFAULT)
    elemType2 = mesh.ElemType(elemCode=SC6R, elemLibrary=STANDARD)
    elemType3 = mesh.ElemType(elemCode=UNKNOWN_TET, elemLibrary=STANDARD)
    cells = p.cells[:]
    pickedRegions =(cells, )
    p.setElementType(regions=pickedRegions, elemTypes=(elemType1, elemType2, elemType3))
    p.seedPart(size=size, deviationFactor=0.1, minSizeFactor=0.1)
    p.generateMesh()


def Create_SPLA(model,instance_name,set_name,load_name,step_name,load):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].sets[set_name]
    mdb.models[model].ConcentratedForce(name=load_name, createStepName=step_name, region=region, cf2=load, distributionType=UNIFORM, field='', localCsys=None)


def Create_Pressure_Load(model,instance_name,load_name,step_name,surface,load):
    a = mdb.models[model].rootAssembly
    region = a.instances[instance_name].surfaces[surface]
    mdb.models[model].Pressure(name=load_name, createStepName=step_name, region=region, distributionType=UNIFORM, field='', magnitude=load, amplitude=UNSET)    

def CreateCutout(model,part,radius_cutout,id_plane,edge,x,y,z):
    p = mdb.models[model].parts[part]
    e, d = p.edges, p.datums
    t = p.MakeSketchTransform(sketchPlane=d[id_plane], sketchUpEdge=edge, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, origin=(x, y, z))
    s = mdb.models[model].ConstrainedSketch(name='__profile__', sheetSize=2000.0, gridSpacing=20.0, transform=t)
    g, v, d1, c = s.geometry, s.vertices, s.dimensions, s.constraints
    s.setPrimaryObject(option=SUPERIMPOSE)
    p.projectReferencesOntoSketch(sketch=s, filter=COPLANAR_EDGES)
    s.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(radius_cutout, 0.0))
    p.CutExtrude(sketchPlane=d[id_plane], sketchUpEdge=edge, sketchPlaneSide=SIDE1, sketchOrientation=RIGHT, sketch=s, flipExtrudeDirection=OFF)
    s.unsetPrimaryObject()
    del mdb.models[model].sketches['__profile__']

def AssignStack(model,part,face):
    p = mdb.models[model].parts[part]
    c = p.cells[:]
    p.assignStackDirection(referenceRegion=face, cells=c)

def CreateJob(model,job_name,cpu):
    a = mdb.models[model].rootAssembly
    mdb.Job(name=job_name, model=model, description='', type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0, queue=None, memory=90, memoryUnits=PERCENTAGE, getMemoryFromAnalysis=True, explicitPrecision=SINGLE, nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF, contactPrint=OFF, historyPrint=OFF, userSubroutine='', scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT, numCpus=cpu, numDomains=cpu, numGPUs=0)

def SubmitJob(job_name):
    mdb.jobs[job_name].submit(consistencyChecking=OFF)

def Linear_Pattern_Instance(model,instance,length,anz):
    a = mdb.models[model].rootAssembly
    a.LinearInstancePattern(instanceList=(instance, ), direction1=(1.0, 0.0, 0.0), direction2=(0.0, 0.0, 1.0), number1=1, number2=anz, spacing1=2740.0, spacing2=length)

def Rotate_Instance(model,instance,x,y,z,thickness,angle):
    a = mdb.models[model].rootAssembly
    a.rotate(instanceList=(instance, ), axisPoint=(x, y, z), 
    axisDirection=(-thickness, 0.0, 0.0), angle=angle)

#----------------------------------------------------------------------------

def Merge_Instance(model,part_new,instance_1,instance_2,instance_3):
    a = mdb.models[model].rootAssembly
    a.InstanceFromBooleanMerge(name=part_new, instances=(a.instances[instance_1], 
    a.instances[instance_2], a.instances[instance_3], ), keepIntersections=ON, originalInstances=SUPPRESS, domain=GEOMETRY)

#----------------------------------------------------------------------------

# variables

myString = "LBA"


myRadius = 1178.2
myThickness = 168.56
myLayerThickness_Facesheet = 0.86
mySandwichCoreThickness = 120.4
myLength = 6880.0
myModel = mdb.Model(name=myString)
myPart = "Cylinder"
myPart_2 = "Hemi_Sphere"
myPart_3 = "Submersible_Shell"
# material parameters

#----------------------------------------------------------------------------

# face sheet 

#----------------------------------------------------------------------------

myE11 = 104060
myE22 = 7396
myE33 = 7396
myNu12 = 0.23
myNu13 = 0.23
myNu23 = 0.34
myG12 = 4042
myG13 = 4042
myG23 = 2666

myLTS = 1918
myLCS = 930
myTTS = 25
myTCS = 86
myLSS = 25
myTSS = 86

#----------------------------------------------------------------------------

# sandwich

#----------------------------------------------------------------------------

myE11_2 = 3715
myE22_2 = 1264
myE33_2 = 1265
myNu12_2 = 0.20
myNu13_2 = 0.20
myNu23_2 = 0.20
myG12_2 = 252
myG13_2 = 180
myG23_2 = 252

myLTS_2 = 49.1
myLCS_2 = 22.14
myTTS_2 = 32
myTCS_2 = 70
myLSS_2 = 14
myTSS_2 = 14

#----------------------------------------------------------------------------

myAngle = [45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,1,
           -45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,-45,45,]

myPlyNumber = len(myAngle)

# hashin damage parameters



Mesh_Size = 86.0


# create model

Create_Part_3D_Cylinder(myRadius,myLength,myThickness,myPart,myString)
Create_Part_3D_Hemi_Sphere(myRadius,myThickness,myPart_2,myString)


myID_1 = Create_Datum_Plane_by_Principal(XZPLANE,myPart,myString,0.0)
myID_2 = Create_Datum_Plane_by_Principal(XYPLANE,myPart,myString,myLength/2.0)
myID_3 = Create_Datum_Plane_by_Principal(YZPLANE,myPart,myString,0.0)
myID_4 = Create_Datum_Plane_by_Principal(XZPLANE,myPart,myString,50.0)

myID_5 = Create_Datum_Plane_by_Principal(XYPLANE,myPart_2,myString,0.0)
myID_6 = Create_Datum_Plane_by_Principal(YZPLANE,myPart_2,myString,0.0)

Create_Set_All_Cells(myString,myPart,"Cylinder_3D")
Create_Set_All_Cells(myString,myPart_2,"Hemi_Sphere_3D")

Create_Set_Face(myRadius-myThickness/2.0,0.0,myLength,myString,myPart,"Set-RP-2")
Create_Set_Face(myRadius-myThickness/2.0,0.0,0.0,myString,myPart,"Set-RP-1")
Create_Set_Face(myRadius,0.0,myLength/2.0,myString,myPart,"External_Surface_cylinder")
Create_Set_Face(0.0,myRadius,0.0,myString,myPart_2,"External_Surface_sphere")
Create_Set_Surface(myRadius,0.0,myLength/2.0,myString,myPart,"External_Surface_cylinder")
Create_Set_Surface(0.0,myRadius,0.0,myString,myPart_2,"External_Surface_sphere")
Create_Set_Surface(myRadius-myThickness,0.0,myLength/2.0,myString,myPart,"Internal_Surface_cylinder")
Create_Set_Surface(0.0,myRadius-myThickness,0.0,myString,myPart_2,"Internal_Surface_sphere")

Create_Material_Data(myString,"CFRP",myE11,myE22,myE33,myNu12,myNu13,myNu23,myG12,myG13,myG23,myLTS,myLCS,myTTS,myTCS,myLSS,myTSS)
Create_Material_Data(myString,"SANDWICH",myE11_2,myE22_2,myE33_2,myNu12_2,myNu13_2,myNu23_2,myG12_2,myG13_2,myG23_2,myLTS_2,myLCS_2,myTTS_2,myTCS_2,myLSS_2,myTSS_2)


Create_Assembly(myString,myPart,"Cylinder-1")
Create_Assembly(myString,myPart_2,"Sphere-1")


#myRP1,myRP_Position1 = Create_Reference_Point(0.0,0.0,0.0,myString,"RP-1")
#myRP2,myRP_Position2 = Create_Reference_Point(0.0,0.0,myLength,myString,"RP-2")

#Create_Constraint_Equation(myString,"Constraint-RP-1","Cylinder-1."+str("Set-RP-1"),"RP-1")
#Create_Constraint_Equation(myString,"Constraint-RP-2","Cylinder-1."+str("Set-RP-2"),"RP-2")

#Create_Analysis_Step(myString,"Step-1","Initial",1.0,1.0,1E-05,100,ON)
#Create_Analysis_Step(myString,"Step-1","Initial",0.01,0.01,1E-015,300,ON)
Create_LBA_Step(myString)

# Create_Boundary_Condition_by_Instance(myString,"Cylinder-1","Set-RP-1","BC-Set-RP-1","Initial",SET,SET,UNSET,UNSET,UNSET,UNSET)
# Create_Boundary_Condition_by_Instance(myString,"Cylinder-1","Set-RP-2","BC-Set-RP-2","Initial",SET,SET,SET,UNSET,UNSET,UNSET)
# Create_Boundary_Condition_by_RP(myString,"RP-1","Displacement_Load","Step-1",UNSET,UNSET,20,UNSET,UNSET,UNSET)


Create_Partion_by_Plane(myString,myPart,myID_3)
myEdge = Create_Set_Edge(0.0,myRadius,myLength/2.0,myString,myPart,"Set-Top-Edge_cylinder")
Create_Partion_by_Plane(myString,myPart,myID_1)
Create_Partion_by_Plane(myString,myPart,myID_2)

Create_Partion_by_Plane(myString,myPart_2,myID_5)
Create_Partion_by_Plane(myString,myPart_2,myID_6)
myEdge_2 = Create_Set_Edge_Multi(0.0,myRadius*np.sin(45.0*np.pi/180.0),myRadius*np.cos(45.0*np.pi/180.0),0.0,myRadius*np.sin(45.0*np.pi/180.0),-myRadius*np.cos(45.0*np.pi/180.0),myString,myPart_2,"Set-Top-Edge_sphere")


Linear_Pattern_Instance(myString,"Sphere-1",myLength,2)
Rotate_Instance(myString,"Sphere-1-lin-1-2",myRadius,0.0,myLength,myThickness,-90.0)
Rotate_Instance(myString,"Sphere-1",myRadius,0.0,0.0,myThickness,90.0)

Merge_Instance(myString,myPart_3,"Cylinder-1","Sphere-1","Sphere-1-lin-1-2")


#Create_Set_Edge(myRadius-myThickness/2.0,0.0,myLength/2.0,myString,myPart,"Set-SPDA-Point")
#Create_Boundary_Condition_by_Instance(myString,"Cylinder-1","Set-SPDA-Point","BC-Imperfection","Step-1",0.0,UNSET,UNSET,UNSET,UNSET,UNSET)
Create_Set_Vertice(0.0,0.0,-myRadius,myString,myPart_3,"BC_Point_back")
Create_Set_Vertice(0.0,0.0,myLength+myRadius,myString,myPart_3,"BC_Point_front")
#Create_SPLA(myString,"Cylinder-1","SPLA_Point","Perturbation_Load","Step-1",-1000.0)
Create_Pressure_Load(myString,"Submersible_Shell-1","External_Pressure_sphere","Step-1","External_Surface_sphere",1)
Create_Pressure_Load(myString,"Submersible_Shell-1","External_Pressure_cylinder","Step-1","External_Surface_cylinder",1)

Create_Boundary_Condition_by_Instance(myString,"Submersible_Shell-1","BC_Point_back","BC_Point_back","Initial",SET,SET,SET,UNSET,UNSET,SET)
Create_Boundary_Condition_by_Instance(myString,"Submersible_Shell-1","BC_Point_front","BC_Point_front","Initial",SET,SET,UNSET,UNSET,UNSET,UNSET)


Create_Sandwich_Composite_Layup(myString,myPart_3,"Cylinder_3D","Quasi_Isotropic",myPlyNumber,"CFRP","SANDWICH",myLayerThickness_Facesheet,mySandwichCoreThickness,myAngle,"External_Surface_cylinder","Set-Top-Edge_cylinder")
Create_Sandwich_Composite_Layup(myString,myPart_3,"Hemi_Sphere_3D","Quasi_Isotropic_2",myPlyNumber,"CFRP","SANDWICH",myLayerThickness_Facesheet,mySandwichCoreThickness,myAngle,"External_Surface_sphere","Set-Top-Edge_sphere")



#CreateCutout(myString,myPart,7.5,myID_4,myEdge,0.0,0.0,myLength/2.0)

Create_Mesh(myString,myPart_3,Mesh_Size)
myFace = Create_Set_Face(myRadius,0.0,myLength/2.0,myString,myPart,"External_Surface_cylinder")
AssignStack(myString,myPart_3,myFace)

CreateJob(myString,"LBA",1)

# SubmitJob("LBA")

#--------------------