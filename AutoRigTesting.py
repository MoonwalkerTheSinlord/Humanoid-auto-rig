import pymel.core as pm
import pymel.core.datatypes as dt
from math import pow,sqrt
from maya import OpenMayaUI as omui
import PySide2
from PySide2 import QtWidgets
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtUiTools import *
from shiboken2 import wrapInstance

import sys
sys.path.append('C:/Users/Sinlord/Desktop')

# =================================================================#
# ======================== GLOBAL VARIABLES =======================#
# =================================================================#
allJoints = pm.ls(type='joint')
sel = pm.ls(sl=True)
ikParentings = []
fkParentings = []
splineParentings = []
drivenParentings = []
if not sel:
	for i in range(50):
		print 'Mesh not selected'

try:
	pm.skinCluster(sel[0].getShape(), ub=True, e=True)
except:
	print ''

rotationOrder = 'xzy'

jointList = []

normX = 1
normY = 0 
normZ = 0
side = "Left"

lyrs = pm.ls(sl=True, type='transform')

#nurbSize = lyrs.worldArea() * 0.1
if lyrs != []:
	bbMinMax = pm.xform(lyrs[0], bb=True, q=True)
	bbMin = bbMinMax[:3]
	bbMax = bbMinMax[3:]
	value = (abs(bbMin[0]) + abs(bbMax[0]) + abs(bbMin[1]) + abs(bbMax[1]) + abs(bbMin[2]) + abs(bbMax[2]))
	radius = value * 0.047
else:
	radius = 1

# =================================================================#
# ===================== DRIVEN FUNCTIONS ==========================#
# =================================================================#
""" ..... Calculate dist ..... """        
def GetDistance(objA, objB):
	gObjA = pm.xform(objA, q=True, t=True, ws=True)
	gObjB = pm.xform(objB, q=True, t=True, ws=True)
	
	return sqrt(pow(gObjA[0]-gObjB[0],2)+pow(gObjA[1]-gObjB[1],2)+pow(gObjA[2]-gObjB[2],2))        

""" ...... Set CTRL positions ..... """                    
def SetCTRLPosition(ctrl, translateX, translateY, translateZ, clr):
    pm.setAttr(ctrl[0] + ".translateX", translateX)
    pm.setAttr(ctrl[0] + ".translateY", translateY)
    pm.setAttr(ctrl[0] + ".translateZ", translateZ)
    
    pm.select(ctrl)
    pm.xform(cp=True)
    pm.makeIdentity(apply=True)
    pm.color(ctrl, ud = clr ) 
    pm.makeIdentity(apply=True)

""" ..... Lock Translation and Scale ..... """
def LockTranslationAndScale(ctrl):
    pm.setAttr(str(ctrl) + '.t', lock=True)
    pm.setAttr(str(ctrl) + '.s', lock=True) 
    
def ParentsingleChainIK(parentCTRL, nrJnts, jntsAPV):
    for i in range(nrJnts - 1):
        pm.parent(jntsAPV[i] + "_IK", parentCTRL)


""" ..... Fill list with joints ..... """ 
def GetJointChildren(node, jntList):
    for child in node.getChildren():

        if child.getChildren() > 0:
            jntList.append(child)

            GetJointChildren(child, jntList)
                 
""" ..... Set driven keys ..... """                  
def SetDrivenKeysValue(jnt, ctrl, valueDownX, valueDownY, valueDownZ, valueUpX, valueUpY, valueUpZ):
    
    #driver value 0
    pm.setDrivenKeyframe(jnt + '.rotateX', cd = str(ctrl) + ".Curl", driverValue = 0, value = 0)
    pm.setDrivenKeyframe(jnt + '.rotateY', cd = str(ctrl) + ".Curl", driverValue = 0, value = 0)
    pm.setDrivenKeyframe(jnt + '.rotateZ', cd = str(ctrl) + ".Curl", driverValue = 0, value = 0)
    
    #driver value 1
    pm.setDrivenKeyframe(jnt + '.rotateX', cd = str(ctrl) + ".Curl", driverValue = 1, value = valueDownX)
    pm.setDrivenKeyframe(jnt + '.rotateY', cd = str(ctrl) + ".Curl", driverValue = 1, value = valueDownY)
    pm.setDrivenKeyframe(jnt + '.rotateZ', cd = str(ctrl) + ".Curl", driverValue = 1, value = valueDownZ)
    
    #driver value -1
    pm.setDrivenKeyframe(jnt + '.rotateX', cd = str(ctrl) + ".Curl", driverValue = -1, value = valueUpX)
    pm.setDrivenKeyframe(jnt + '.rotateY', cd = str(ctrl) + ".Curl", driverValue = -1, value = valueUpY)
    pm.setDrivenKeyframe(jnt + '.rotateZ', cd = str(ctrl) + ".Curl", driverValue = -1, value = valueUpZ)

          
# =================================================================#
# ======================== finger_ctrl ============================#
# =================================================================#

def RunDriven(wristJoint, jntList):
	del jntList[:]
	jointList.append(wristJoint)
	GetJointChildren(wristJoint, jntList)
	
	MiddleFingerKnucklePos = jntList[9].getTranslation(worldSpace=True)
	
	distX = GetSide(jntList[1], wristJoint, 0)
	distZ = GetSide(jntList[1], wristJoint, 2)
	
	vector = GetSide(jntList[1],  jntList[11], 0)
	
	#0 deg
	if distX > 0 and distZ > 0 and vector < 0:
	    side = "Left"
	
	if distX < 0 and distZ > 0 and vector > 0: 
	    side = "Right"  
	    
	#90 deg 
	if distX > 0 and distZ < 0 and vector > 0:
	    side = "Left"
	        
	if distX > 0 and distZ > 0 and vector > 0:    
	    side = "Right" 
	       
	#180 deg 
	if distX < 0 and distZ < 0 and vector > 0:
	    side = "Left"
	        
	if distX > 0 and distZ < 0 and vector < 0:      
	    side = "Right" 
	    
	#270 deg 
	if distX < 0 and distZ > 0 and vector < 0:
	    side = "Left"
	    
	if distX < 0 and distZ < 0 and vector < 0:  
	    side = "Right" 
	
	# ========= CREATE CONTROLLER ========= #
	CTRL = pm.circle(name = jntList[9] + '_ctrl', normal=(0, 1, 0), c=(0, 0 , 0), r = 0.2)
	groupings = CTRL[0]
	pm.setAttr(str(jntList[9]) + "_ctrl.translateX", MiddleFingerKnucklePos.x)
	pm.setAttr(str(jntList[9]) + "_ctrl.translateY", MiddleFingerKnucklePos.y + 0.15)
	pm.setAttr(str(jntList[9]) + "_ctrl.translateZ", MiddleFingerKnucklePos.z)
		
	pm.select(CTRL)
	pm.xform(cp=True)
	pm.makeIdentity(apply=True)
	pm.color(jntList[9] + '_ctrl', ud = 6)
		
	pm.addAttr(jntList[9] + '_ctrl', shortName = 'crl', longName = 'Curl', keyable = True, defaultValue = 0.0, minValue = -1.00, maxValue = 1.0 )
	
	# ========= THUMB ========= #
	if side == "Left":    
		    SetDrivenKeysValue(jntList[1], CTRL[0], 0, -20, -2, 0, 0, 40)
		    SetDrivenKeysValue(jntList[2], CTRL[0], 0, 0, -5, 0, 0, 30)
		    SetDrivenKeysValue(jntList[3], CTRL[0], 0, 0, -65, 0, 0, 25)
		      
	elif side == "Right":     
		    SetDrivenKeysValue(jntList[1], CTRL[0], 0, 20, -2, 0, 0, 40)
		    SetDrivenKeysValue(jntList[2], CTRL[0], 0, 0, -5, 0, 0, 30)
		    SetDrivenKeysValue(jntList[3], CTRL[0], 0, 0, -65, 0, 0, 20)
		
	# ====== INDEX FINGER ====== # 
	if side == "Left":
	    SetDrivenKeysValue(jntList[5], CTRL[0], 5, -90, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[6], CTRL[0], 0, -95, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[7], CTRL[0], 0, -95, 0, 0, 6, 0)
		         
	elif side == "Right":
	    SetDrivenKeysValue(jntList[5], CTRL[0], 5, 90, 0, 0, -7, 0)
	    SetDrivenKeysValue(jntList[6], CTRL[0], 0, 95, 0, 0, -7, 0)
	    SetDrivenKeysValue(jntList[7], CTRL[0], 0, 95, 0, 0, -6, 0)
		
	# ====== MIDDLE FINGER ====== # 
	if side == "Left":
	    SetDrivenKeysValue(jntList[9], CTRL[0], 0, -90, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[10], CTRL[0], 0, -95, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[11], CTRL[0], 0, -95, 0, 0, 6, 0)
		        
	elif side == "Right":
	    SetDrivenKeysValue(jntList[9], CTRL[0], 0, 90, 0, 0, -7, 0)
	    SetDrivenKeysValue(jntList[10], CTRL[0], 0, 95, 0, 0, -7, 0)
	    SetDrivenKeysValue(jntList[11], CTRL[0], 0, 95, 0, 0, -6, 0)
		       
	# ====== RING FINGER ====== #      
	if side == "Left":
	    SetDrivenKeysValue(jntList[13], CTRL[0], -5, -90, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[14], CTRL[0], 0, -95, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[15], CTRL[0], 0, -95, 0, 0, 6, 0)
		       
	elif side == "Right":
	    SetDrivenKeysValue(jntList[13], CTRL[0], 5, 90, 0, 0, -7, 0)
	    SetDrivenKeysValue(jntList[14], CTRL[0], 0, 95, 0, 0, -7, 0)
	    SetDrivenKeysValue(jntList[15], CTRL[0], 0, 95, 0, 0, -6, 0)
		 
	# ====== PINKY FINGER ====== #
	if side == "Left":
	    SetDrivenKeysValue(jntList[17], CTRL[0], -10, -90, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[18], CTRL[0], 0, -95, 0, 0, 7, 0)
	    SetDrivenKeysValue(jntList[19], CTRL[0], 0, -95, 0, 0, 6, 0)
		      
	elif side == "Right":     
		 SetDrivenKeysValue(jntList[17], CTRL[0], 10, 90, 0, 0, -7, 0)
		 SetDrivenKeysValue(jntList[18], CTRL[0], 0, 95, 0, 0, -7, 0)
		 SetDrivenKeysValue(jntList[19], CTRL[0], 0, 95, 0, 0, -6, 0)
		
		             
	pm.select(CTRL)
	pm.setAttr(CTRL[0] + '.s', lock=True)
	pm.setAttr(CTRL[0] + '.t', lock=True)
	pm.setAttr(CTRL[0] + '.r', lock=True)
		
		
	if pm.objExists(str(jntList[0]) + '_CTRL'):
	    pm.parent(CTRL[0], str(jntList[0]) + '_CTRL')
	return groupings

    
# =================================================================#
# ========================= IK FUNCTIONS ==========================#
# =================================================================#

def SetIK(startJoint, poleVectorJoint, endJoint, jntList, hP, tTap, tTip, ankle):
	del jntList[:]
	jntList.append(endJoint)
	GetJointParents(endJoint, jntList)
	groupings = []
	jntList = jntList[::-1]
	
	# remove unnessesary joints (above start joint)
	jntIndex = 0
	for ind in range(len(jntList)):
		if str(jntList[ind]) == startJoint:
			jntIndex = ind		
			
	jntList = jntList[jntIndex + 1::]
	nrOfJoints = len(jntList)
	
	#find pole vector jnt index	
	poleVIndex = 0
	for i in range(len(jntList)):
		if str(jntList[i]) == poleVectorJoint:
			poleVIndex = i
	
	jntsAfterPV = jntList[poleVIndex::]
	nrOfJnt = len(jntsAfterPV)
	
	
	OrientJoints(jntList)
	OrientEndJoints(jntList)
	
	# ======================= JNT POSITIONS =========================== #   
	endJointIndex = jntList.index(endJoint)
		 
	startJointPosition = startJoint.getTranslation(worldSpace=True)
	poleVectorJointPosition = poleVectorJoint.getTranslation(worldSpace=True)
	
	if (endJointIndex - 1) == poleVIndex:
	    footJointIndex = endJointIndex
	    footJointPosition = jntList[endJointIndex].getTranslation(worldSpace=True)
	    
	elif (endJointIndex - 1) != poleVIndex:
	    footJointIndex = endJointIndex - 1
	    footJointPosition = jntList[endJointIndex - 1].getTranslation(worldSpace=True)
	    	    
	    
	endJointPosition = endJoint.getTranslation(worldSpace=True)
	
	# ======================= CREATE IK =============================== #     
	
	rotatePlaneSolverIK = pm.ikHandle(name = startJoint + "_IK", sj = startJoint, ee = poleVectorJoint, sol = 'ikRPsolver')  
	
	for i in range(nrOfJnt - 1):
	    singleChainIK = pm.ikHandle(name = jntsAfterPV[i] + "_IK", sj = jntList[poleVIndex + i], ee = jntList[poleVIndex + i + 1], sol = 'ikSCsolver')

  
	# =================================================================#
	# ====================== CONTROLERS ===============================#
	# =================================================================#     
	
	# ======================= FOOT CTRL ============================== #
	
	mainCtrl = pm.circle(name=startJoint + '_CTRL', normal=(0, 1, 0), c=(0, 0 , 0), r = radius )
	groupings.append(mainCtrl)
	SetCTRLPosition(mainCtrl, poleVectorJointPosition.x, bbMinMax[1], poleVectorJointPosition.z + (endJointPosition.z / 2), 3)
	pm.setAttr(mainCtrl[0] + '.s', lock=True)
	
	pm.parent(rotatePlaneSolverIK[0], mainCtrl[0])
	
	ParentsingleChainIK(mainCtrl[0], nrOfJnt, jntsAfterPV)
	
	# ======================= POLE VECTOR ============================= #
	
	distance = GetDistance(poleVectorJoint, startJoint)
	halfDistance = distance / 2
	poleVector = pm.circle(name = poleVectorJoint + '_CTRL', normal=(0, 0, 1), c=(0, 0 ,0), r=radius * 0.5 )
	
	SetCTRLPosition(poleVector, poleVectorJointPosition.x, poleVectorJointPosition.y + halfDistance, endJointPosition.z * 5, 3)
	pm.poleVectorConstraint(poleVector, rotatePlaneSolverIK[0])
	
	pm.setAttr(poleVectorJoint + '_CTRL.s', lock=True)
	pm.setAttr(poleVectorJoint + '_CTRL.r', lock=True)
	pm.setAttr(poleVectorJoint + '_CTRL.ty', lock=True)
	pm.setAttr(poleVectorJoint + '_CTRL.tz', lock=True)
	
	pm.parent(poleVector[0], mainCtrl[0])
	
		

	# ===================== "TOE" TAP CTRL ============================ #
	if (tTap):
		tapCTRL = pm.circle(name= jntList[footJointIndex] + '_tap_CTRL', normal=(1, 0, 0), c=(0,0,0), r=radius * 0.3 )
		
		SetCTRLPosition(tapCTRL, poleVectorJointPosition.x, footJointPosition.y, footJointPosition.z, 4) 
		
		LockTranslationAndScale(jntList[footJointIndex] + '_tap_CTRL')
		pm.setAttr(jntList[footJointIndex] + '_tap_CTRL.ry', lock=True)
		pm.setAttr(jntList[footJointIndex] + '_tap_CTRL.rz', lock=True)
		
		ParentsingleChainIK(tapCTRL[0], nrOfJnt, jntsAfterPV)
		pm.parent(tapCTRL[0], mainCtrl[0])
	  
	
	# ==================== "HEEL" PEEL CTRL ============================= #
	if (hP):
		peelCTRL = pm.circle(name=jntList[footJointIndex] + '_peel_CTRL', normal=(1, 0, 0), c=(0,0,0), r= radius * 0.4 )
		
		SetCTRLPosition(peelCTRL, poleVectorJointPosition.x, footJointPosition.y, footJointPosition.z, 5) 
		
		LockTranslationAndScale(jntList[footJointIndex] + '_peel_CTRL')
		pm.setAttr(jntList[footJointIndex] + '_peel_CTRL.ry', lock=True)
		pm.setAttr(jntList[footJointIndex] + '_peel_CTRL.rz', lock=True)
		
		pm.parent(rotatePlaneSolverIK[0], peelCTRL[0])
		pm.parent(peelCTRL[0], mainCtrl[0])
	
	# ================== MOVE MAIN CTRL PIVOT ========================== #
	pm.select(mainCtrl) 
	pm.move(poleVectorJointPosition.x, footJointPosition.y, poleVectorJointPosition.z, str(startJoint) + "_CTRL.scalePivot",str(startJoint) + "_CTRL.rotatePivot", absolute=True)
	
	# ======================= SWIVEL CTRL ============================== #
	if (hP and tTap):
		swivelCTRL = pm.circle(name=jntList[footJointIndex] + '_swivel_CTRL', normal=(0, 1, 0), c=(0, 0, 0), r=radius * 0.5 )
		
		SetCTRLPosition(swivelCTRL, poleVectorJointPosition.x, footJointPosition.y, footJointPosition.z, 5) 
		
		LockTranslationAndScale(jntList[footJointIndex] + '_swivel_CTRL')
		pm.setAttr(jntList[footJointIndex] + '_swivel_CTRL.rx', lock=True)
		pm.setAttr(jntList[footJointIndex] + '_swivel_CTRL.rz', lock=True)
		
		pm.parent(peelCTRL[0], swivelCTRL[0])
		pm.parent(tapCTRL[0], swivelCTRL[0])
		pm.parent(swivelCTRL[0], mainCtrl[0])
	
	 
	# ==========================  TIP CTRL ============================== #
	if (tTip):
		tipCTRL = pm.circle(name= jntList[endJointIndex] + '_end_CTRL', normal=(1, 0, 0), c=(0,0,0), r=radius * 0.2 )
		
		SetCTRLPosition(tipCTRL, poleVectorJointPosition.x, endJointPosition.y, endJointPosition.z + 0.1, 8) 
		
		LockTranslationAndScale(jntList[endJointIndex] + '_end_CTRL')
		pm.setAttr(jntList[endJointIndex] + '_end_CTRL.ry', lock=True)
		pm.setAttr(jntList[endJointIndex] + '_end_CTRL.rz', lock=True)
		
		pm.parent(swivelCTRL[0], tipCTRL[0])
		pm.parent(tipCTRL[0], mainCtrl[0])
	
	
	# ======================= "ANKLE" CTRL ============================== #
	if (ankle):
		poleJointCTRL = pm.circle(name = jntList[endJointIndex] + '_poleJoint_CTRL', normal=(1, 0, 0), c=(0,0,0), r=radius * 0.5 )
		
		SetCTRLPosition(poleJointCTRL, poleVectorJointPosition.x, poleVectorJointPosition.y, poleVectorJointPosition.z, 8) 
		
		LockTranslationAndScale(jntList[endJointIndex] + '_poleJoint_CTRL')
		pm.setAttr(jntList[endJointIndex] + '_poleJoint_CTRL.ry', lock=True)
		pm.setAttr(jntList[endJointIndex] + '_poleJoint_CTRL.rz', lock=True)
		
		pm.parent(tipCTRL[0], poleJointCTRL[0])
		pm.parent(poleJointCTRL[0], mainCtrl[0])
		
	return groupings
# =================================================================#
# ===================== ALL-AROUND FUNCTIONS ======================#
# =================================================================#

""" ..... Set Orientation for end joints ..... """                
def OrientEndJoints(jntList):
    
    for jnt in jntList:
        kids = jnt.getChildren()
        childLen = len(kids)
        if childLen <= 0:
            pm.setAttr(jnt + ".jointOrientX", 0)
            pm.setAttr(jnt + ".jointOrientY", 0)
            pm.setAttr(jnt + ".jointOrientZ", 0)


""" ..... Set Orientations for joints ..... """            
def OrientJoints(jntList):
    
    for jnts in jntList:
        pm.select(jnts)
        pm.makeIdentity(apply=True)
        pm.joint(jnts, e=True, oj = 'xyz', sao = 'zup', ch = True, zso = True)
        
        pm.select(jnts)   
        pm.makeIdentity(apply=True)


def FormList( allJnts, widgetList ):
	newList = []
	counter = widgetList.count()
	if (counter > 0):
		for i in range(counter):
			item = widgetList.item(i)
			for jnt in allJnts:
				if (str(item.text()) == str(jnt)):
					newList.append(jnt)
	return newList
	

	
""" ..... Fill list with joints ..... """             
def GetJointParents(node, jntList):
    if node.getParent() > 0:
        prnt = node.getParent()
        jntList.append(prnt)
        
        GetJointParents(prnt, jntList)



""" ..... Parent controllers ..... """     
def ParentControllers(jntPName, jointCName):
    pm.parent(str(jntPName) + '_GRP', str(jointCName) + '_CTRL')



def LockAndHide(*args):
    for i in range(1, len(args)):
        pm.setAttr(args[0] + args[i][0], lock=True, keyable=False, channelBox=False)
        pm.setAttr(args[0] + args[i][1], lock=True, keyable=False, channelBox=False)
        pm.setAttr(args[0] + args[i][2], lock=True, keyable=False, channelBox=False)
        

""" ..... Get joints axis ..... """
def GetAxis(objA, objB):
    ObjA = pm.xform(objA, q=True, t=True, ws=True)
    ObjB = pm.xform(objB, q=True, t=True, ws=True)
    
    axis = 0
    highestValue = 0
    
    for x in range(3):
        AB = dt.abs(ObjA[x] - ObjB[x])
        
        if AB > highestValue:
            highestValue = AB
            axis = x
            
    return axis

""" ..... Calculate dist vector..... """  
def GetSide(objA, objB, x):
    ObjA = pm.xform(objA, q=True, t=True, ws=True)
    ObjB = pm.xform(objB, q=True, t=True, ws=True)
        
    dist = ObjA[x] - ObjB[x]
            
    return dist  
    
def findLastFKChild(item):
	newChild = item.listRelatives()[0]
	try:
		if (newChild.listRelatives()[1]):
			newChild = newChild.listRelatives()[1]
			findLastChild(newChild)
	except:
		return newChild.getParent()


def completeHierarchy(sP, fP, iP, dP):
	if (len(fP) > 0):
		for fk in fP:
			pm.parent(fk[0], sP[0])
	if dP:
		if (len(dP) > 0):
			pm.parent(dP[0], dP[1])
			pm.parent(dP[3], dP[5])
	if (len(iP) > 0):
		for ik in iP:
			pm.parent(ik[0][0], sP[1])


# =================================================================#
# ========================== FK FUNCTIONS =========================#
# =================================================================#
            
""" ..... Create controllers for joints ..... """             
def SetFKController(jntName, jointNr, jntAxis, size, grp):
	if jntAxis == 0:
		ctrl = pm.circle(name=jntName[jointNr] + '_CTRL', normal=(1, 0, 0), c=(0, 0 , 0), r = size)
	elif jntAxis == 1:
		ctrl = pm.circle(name=jntName[jointNr] + '_CTRL', normal=(0, 1, 0), c=(0, 0 , 0), r = size)
	elif jntAxis == 2:
		ctrl = pm.circle(name=jntName[jointNr] + '_CTRL', normal=(0, 0, 1), c=(0, 0 , 0), r = size)
			
	GRP = pm.group(em=True, name= jntName[jointNr] + '_GRP')
	if (jointNr == 0):
		grp.append(GRP)
	if (jointNr == len(jntName) - 1):
		grp.append(GRP)
	pm.parent(ctrl[0], GRP)
	pm.select(GRP)
	
	pm.parent(GRP, jntName[jointNr])
	
	pm.setAttr(str(jntName[jointNr]) + "_GRP.translateX", 0)
	pm.setAttr(str(jntName[jointNr]) + "_GRP.translateY", 0)
	pm.setAttr(str(jntName[jointNr]) + "_GRP.translateZ", 0)
	
	
	pm.xform(cp=True)
	pm.makeIdentity(apply=True)
	pm.color(jntName[jointNr] + '_CTRL', ud = 4)
	
	pm.parent(GRP, world=True )
	pm.select(GRP)
	pm.makeIdentity(apply=True)
	
	pm.orientConstraint(ctrl, jntName[jointNr], mo=True)
	
	pm.setAttr(str(jntName[jointNr]) +'_CTRL.t', lock=True)
	pm.setAttr(str(jntName[jointNr]) +'_CTRL.s', lock=True)
	
	pm.setAttr(str(jntName[jointNr]) +'_GRP.t', lock=True)
	pm.setAttr(str(jntName[jointNr]) +'_GRP.s', lock=True)
	pm.setAttr(str(jntName[jointNr]) +'_GRP.r', lock=True)
	        

def SetFK(startJoint, endJoint, jntList):
	del jntList[:]
	groupings = []
	size = radius
	jntList.append(endJoint)
	GetJointParents(endJoint, jntList)
	   
	jntList = jntList[::-1]
	jntIndex = 0
	
	# remove unnessesary joints (above start joint)
	for ind in range(len(jntList)):
	    if str(jntList[ind]) == startJoint:
	        jntIndex = ind	
	        
	jntList = jntList[jntIndex::]
	nrOfJoints = len(jntList)  
	axis = GetAxis(startJoint, endJoint)
	
	for index in range(nrOfJoints):
	    SetFKController(jntList, index, axis, size, groupings)
	    size = size * 0.8
	   
	
	for index in range(nrOfJoints - 1):
		ParentControllers(jntList[index + 1], jntList[index])   
		   
	return groupings
    
# =================================================================#
# ====================== SPLINE FUNCTIONS =========================#
# =================================================================#

    
def SquashAndStretch(splineCurve, jntList):
    pm.select(splineCurve)
    curveInfoNode = pm.shadingNode('curveInfo', asUtility=True, name='spineInfo')
    splineCurveShape = splineCurve.listRelatives(type='shape')[0]
    pm.connectAttr(splineCurveShape + '.worldSpace[0]', curveInfoNode + '.inputCurve', force=True)
    spineStretchPercent = pm.shadingNode('multiplyDivide', asUtility=True, name='spine_stretch_percent_div')
    pm.connectAttr(curveInfoNode + '.arcLength', spineStretchPercent + '.input1X', force=True)
    arcLength = pm.getAttr(curveInfoNode + '.arcLength')
    pm.setAttr(spineStretchPercent + '.input2X', arcLength)
    pm.setAttr(spineStretchPercent + '.operation', 2)
    for jnt in jntList:
        pm.connectAttr(spineStretchPercent + '.outputX', jnt.scaleX, force=True)
    
    sqrtStretchPow = pm.shadingNode('multiplyDivide', asUtility=True, name='spine_sqrt_stretch_pow')
    pm.connectAttr(spineStretchPercent + '.outputX', sqrtStretchPow + '.input1X', force=True)
    pm.setAttr(sqrtStretchPow + '.operation', 3)
    pm.setAttr(sqrtStretchPow + '.input2X', 0.5)
    
    stretchInvertDiv = pm.shadingNode('multiplyDivide', asUtility=True, name='spine_stretch_invert_div')
    pm.connectAttr(sqrtStretchPow + '.outputX', stretchInvertDiv + '.input2X', force=True)
    pm.setAttr(stretchInvertDiv + '.operation', 2)
    pm.setAttr(stretchInvertDiv + '.input1X', 1)
    
    for jnt in jntList:
        pm.connectAttr(stretchInvertDiv + '.outputX', jnt.scaleY, force=True)
        pm.connectAttr(stretchInvertDiv + '.outputX', jnt.scaleZ, force=True)
    
    
    
def CreateFKSpineControl(targetNode, name):
    spine1FkControl = pm.circle(normal=(1, 0, 0), sweep=360, r=radius)[0]
    pm.rename(spine1FkControl, name)
    pm.makeIdentity(spine1FkControl, apply=True)
    spine1ShapeNode = spine1FkControl.listRelatives(type='shape')[0]
    pm.parent(spine1ShapeNode, targetNode, shape=True, relative=True)
    pm.delete(spine1FkControl)

def duplicateAndRename(list, last, reName, rotOrder):
    startJoint = pm.duplicate(list[0], po=True, st=True)[0]
    endJoint = pm.duplicate(list[last], po=True, st=True)[0]
    pm.makeIdentity(startJoint)
    pm.makeIdentity(endJoint)
    pm.parent(endJoint, w=True)
    pm.rename(startJoint, list[0] + reName)
    pm.rename(endJoint, list[last] + reName)    
    
    return startJoint, endJoint
    
    
def fkSpineControl(firstJnt, secondJnt, rotOrder):
    jnt1Trans = firstJnt.getTranslation(space='world')
    jnt2Trans = secondJnt.getTranslation(space='world')
    distanceCurve = pm.curve(d=1, p=[(jnt1Trans), (jnt2Trans)], k=[0, 1])
    firstPointInSpace = pm.pointPosition(distanceCurve + '.u[0.33]', world=True)
    secondPointInSpace = pm.pointPosition(distanceCurve + '.u[0.67]', world=True)
    pm.delete(distanceCurve)
    pm.select(d=True)
    midJnt1 = pm.duplicate(secondJnt, po=True, st=True)[0]
    midJnt2 = pm.duplicate(secondJnt, po=True, st=True)[0]
    midJnt1.setTranslation(firstPointInSpace, space='world')
    midJnt2.setTranslation(secondPointInSpace, space='world')
    
    return midJnt1, midJnt2


def TwistControl(ikHandl, startJ, endJ):
    pm.setAttr(str(ikHandl) + '.dTwistControlEnable', 1)
    pm.setAttr(str(ikHandl) + '.dWorldUpType', 4)
    pm.setAttr(str(ikHandl) + '.dForwardAxis', 0)
    pm.setAttr(str(ikHandl) + '.dWorldUpAxis', 0)
    pm.connectAttr(startJ.worldMatrix[0], ikHandl.dWorldUpMatrix, force=True)
    pm.connectAttr(endJ.worldMatrix[0], ikHandl.dWorldUpMatrixEnd, force=True)
    
    
def CreateIkSpline(listOfJoints, rotOrder, twist, sns):
    groupings = []
    lastElement = len(listOfJoints) - 1
    pm.select(listOfJoints[0])
    pm.select(listOfJoints[lastElement], add=True)
    ik = pm.ikHandle(solver='ikSplineSolver', name='spine_hdl')
    pm.rename(ik[1], 'spine_effector')
    pm.rename(ik[2], 'spine_crv')

    ikJoints = duplicateAndRename(listOfJoints, lastElement, '_bind', rotOrder)
    
    pm.skinCluster(ikJoints[0], ikJoints[1], ik[2], mi=2)
    hipBox = pm.curve(d=1, p=[(0.5, 0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5), (-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5)], k=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17])
    hipTranslations = pm.getAttr(listOfJoints[0] + '.translate')
    pm.move(hipTranslations)
    pm.scale(hipBox, [radius, radius, radius])
    pm.makeIdentity(hipBox, apply=True)
    endBox = pm.curve(d=1, p=[(0.5, 0.5, -0.5), (-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5), (0.5, -0.5, -0.5), (0.5, 0.5, -0.5), (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5), (-0.5, 0.5, -0.5), (-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5), (-0.5, 0.5, 0.5), (0.5, 0.5, 0.5), (0.5, -0.5, 0.5), (-0.5, -0.5, 0.5)], k=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17])
    groupings.append(endBox)
    endJointTranslation = listOfJoints[lastElement].getTranslation(space='world')
    pm.move(endJointTranslation)
    pm.scale(endBox, [radius, radius, radius])
    pm.makeIdentity(endBox, apply=True)
    pm.setAttr(str(hipBox) + '.rotateOrder', rotOrder)
    pm.setAttr(str(endBox) + '.rotateOrder', rotOrder)
    pm.rename(hipBox, listOfJoints[0] + '_ctrl')
    pm.rename(endBox, listOfJoints[lastElement] + '_ctrl')
    pm.setAttr(str(ikJoints[0]) + '.rotateOrder', rotOrder)
    pm.setAttr(str(ikJoints[1]) + '.rotateOrder', rotOrder)
    pm.parentConstraint(str(endBox), str(ikJoints[1]), mo=True)
    pm.parentConstraint(str(hipBox), str(ikJoints[0]), mo=True)
    
    if (twist):
        TwistControl(ik[0], ikJoints[0], ikJoints[1])
    
    fkJoints = duplicateAndRename(listOfJoints, lastElement, '_fk_jnt', rotOrder)
    fkMidJnts = fkSpineControl(fkJoints[0], fkJoints[1], rotOrder)
    pm.rename(fkMidJnts[0], 'spine1_fk_ctrl')
    pm.rename(fkMidJnts[1], 'spine2_fk_ctrl')
    
    pm.parent(fkMidJnts[0], fkJoints[0])
    pm.parent(fkMidJnts[1], fkMidJnts[0])
    pm.parent(fkJoints[1], fkMidJnts[1])
    
    fkChain = [ fkJoints[0], fkMidJnts[0], fkMidJnts[1], fkJoints[1] ]
    hipConstGrp = pm.group(hipBox, name=(listOfJoints[0] + '_fkConst_grp'))
    lastJntConstGrp = pm.group(endBox, name=(listOfJoints[lastElement] + '_fkConst_grp'))
    pm.parentConstraint(fkChain[3], lastJntConstGrp, mo=True)
    pm.parentConstraint(fkChain[0], hipConstGrp, mo=True)
    
    CreateFKSpineControl(fkChain[1], 'spine1_fk_ctrl')
    CreateFKSpineControl(fkChain[2], 'spine2_fk_ctrl')
    pm.select(fkChain[0])
    pm.joint(edit=True, oj='xyz', secondaryAxisOrient='zup', ch=True, zso=True)
    
    pm.select(hipBox)
    pm.select(endBox, add=True)
    torsoIKDisplay = pm.createDisplayLayer(name='torso_ik_layer', number=1, nr=True)
    pm.setAttr(torsoIKDisplay + '.color', 17)
    pm.select(fkMidJnts[0])
    pm.select(fkMidJnts[1], add=True)
    torsoFKDisplay = pm.createDisplayLayer(name='torso_fk_layer', number=1, nr=True)
    pm.setAttr(torsoFKDisplay + '.color', 15)
    newList = listOfJoints[:-1]
    
    if (sns):
        SquashAndStretch(ik[2], newList)
    
    bodyCurve = pm.curve(d=1, p=[(-1.0, 0, 1.0), (-1.0, 0, -1.0), (1.0, 0, -1.0), (1.0, 0, 1.0), (-1.0, 0, 1.0)], k=[0, 1, 2, 3, 4])
    pm.xform(bodyCurve, cp=True)
    pm.setAttr(bodyCurve + '.scale', (radius * 3, radius * 3, radius * 3))
    pm.setAttr(bodyCurve + '.translate', hipTranslations)
    pm.rename(bodyCurve, 'body_ctrl')
    pm.setAttr(bodyCurve + '.rotateOrder', rotOrder)
    pm.makeIdentity(bodyCurve, apply=True)
    torsoGrp = pm.group(ik[0], ik[2], ikJoints[0], ikJoints[1], fkChain[0], hipConstGrp, lastJntConstGrp, name='torso_grp')
    groupings.append(torsoGrp)
    pm.parentConstraint(bodyCurve, torsoGrp, mo=True)
    pm.setAttr(ik[2] + '.inheritsTransform', 0)
    
    pm.select(bodyCurve)
    bodyDisplay = pm.createDisplayLayer(name='body_layer', number=1, nr=True)
    pm.setAttr(bodyDisplay + '.color', 13)
    
    untouchableGrp = pm.group(ik[0], ik[2], ikJoints[0], ikJoints[1], listOfJoints[0], p=torsoGrp, name='DO_NOT_TOUCH_GRP')
    pm.setAttr(fkMidJnts[0] + '.radi', lock=True, keyable=False, channelBox=False)
    pm.setAttr(fkMidJnts[1] + '.radi', lock=True, keyable=False, channelBox=False)
    scaleXYZ = ['.sx', '.sy', '.sz']
    transXYZ = ['.tx', '.ty', '.tz']
    rotXYZ = ['.rx', '.ry', '.rz']
    LockAndHide(bodyCurve, scaleXYZ)
    LockAndHide(endBox, scaleXYZ)
    #LockAndHide(hipBox, scaleXYZ)
    LockAndHide(fkMidJnts[0], scaleXYZ, transXYZ)
    LockAndHide(fkMidJnts[1], scaleXYZ, transXYZ)
    LockAndHide(fkChain[0], scaleXYZ, transXYZ, rotXYZ)
    LockAndHide(fkChain[3], scaleXYZ, transXYZ, rotXYZ)
    LockAndHide(hipConstGrp, scaleXYZ, transXYZ, rotXYZ)
    LockAndHide(lastJntConstGrp, scaleXYZ, transXYZ, rotXYZ)
    LockAndHide(torsoGrp, scaleXYZ, transXYZ, rotXYZ)
    
    rootTransformGrp = pm.group(bodyCurve, torsoGrp, name='root_transform_grp')
    pm.setAttr(hipBox + '.rotateZ', -90)
    




######################################UI############################################
def getMayaWin():
	mayaWinPtr = omui.MQtUtil.mainWindow( )
	mayaWin = wrapInstance( long( mayaWinPtr ), QtWidgets.QMainWindow )


def loadUI( path ):
	loader = QUiLoader()
	uiFile = QFile( path )
	
	dirIconShapes = ""
	buff = None
	
	if uiFile.exists():
		dirIconShapes = path
		uiFile.open( QFile.ReadOnly )
		
		buff = QByteArray( uiFile.readAll() )
		uiFile.close()
	else:
		print "UI file missing! Exiting..."
		exit(-1)

	fixXML( path, buff )
	qbuff = QBuffer()
	qbuff.open( QBuffer.ReadOnly | QBuffer.WriteOnly )
	qbuff.write( buff )
	qbuff.seek( 0 )
	ui = loader.load( qbuff, parentWidget = getMayaWin() )
	ui.path = path
	
	return ui


def fixXML( path, qbyteArray ):
	# first replace forward slashes for backslashes
	if path[-1] != '/':
		path += '/'
	path = path.replace( "/", "\\" )

	# construct whole new path with <pixmap> at the begining
	tempArr = QByteArray( "<pixmap>" + path + "\\" )

	# search for the word <pixmap>
	lastPos = qbyteArray.indexOf( "<pixmap>", 0 )
	while lastPos != -1:
		qbyteArray.replace( lastPos, len( "<pixmap>" ), tempArr )
		lastPos = qbyteArray.indexOf( "<pixmap>", lastPos + 1 )
	return


class UIController:	
	def __init__( self, ui ):
        
		ui.rigIt.clicked.connect( self.RunRig )	
		ui.upSpline.clicked.connect( lambda: self.MoveUpItem(ui.listSpline) )
		ui.upIK.clicked.connect( lambda: self.MoveUpItem(ui.listIK) )
		ui.upFK.clicked.connect( lambda: self.MoveUpItem(ui.listFK) )
		ui.downSpline.clicked.connect( lambda: self.MoveDownItem(ui.listSpline) )
		ui.downIK.clicked.connect( lambda: self.MoveDownItem(ui.listIK) )
		ui.downFK.clicked.connect( lambda: self.MoveDownItem(ui.listFK) )
		ui.addSpline.clicked.connect( lambda: self.AddToWidgetList(ui.listSpline) )
		ui.addFK.clicked.connect( lambda: self.AddToWidgetList(ui.listFK) )
		ui.addDriven.clicked.connect( lambda: self.AddToWidgetList(ui.listDriven) )
		ui.addIK.clicked.connect( lambda: self.AddToWidgetList(ui.listIK) )
		ui.removeIK.clicked.connect( lambda: self.RemoveItem(ui.listIK) )
		ui.removeFK.clicked.connect( lambda: self.RemoveItem(ui.listFK) )		
		ui.removeDriven.clicked.connect( lambda: self.RemoveItem(ui.listDriven) )
		ui.removeSpline.clicked.connect( lambda: self.RemoveItem(ui.listSpline) )
		ui.drivenKeys.clicked.connect( self.ToggleDrivenKeys )
		        
		self.ui = ui
		ui.setWindowFlags( Qt.WindowStaysOnTopHint )
		ui.show()

	def ToggleDrivenKeys( self ):
		if ( ui.drivenKeys.isChecked() ):
			ui.addDriven.setEnabled(True)
			ui.removeDriven.setEnabled(True)
			ui.listDriven.setEnabled(True)
		else:
			ui.addDriven.setEnabled(False)
			ui.removeDriven.setEnabled(False)
			ui.listDriven.setEnabled(False)
	
	def AddToWidgetList( self, widgetList):
		newSelect = pm.ls(sl=True)
		if ( len(newSelect) > 1 ):
			for jnt in newSelect:
				widgetList.addItem(str(jnt))
		else:
			widgetList.addItem(str(newSelect[0]))

	def RemoveItem( self, widgetList ):
		index = widgetList.currentRow()
		item = widgetList.takeItem(index)
		item = None

	def MoveUpItem( self, widgetList ):
		index = widgetList.currentRow()
		item = widgetList.takeItem(index)
		widgetList.insertItem(index - 1, item)
		widgetList.setCurrentRow(index - 1)

	def MoveDownItem( self, widgetList ):
		index = widgetList.currentRow()
		item = widgetList.takeItem(index)
		widgetList.insertItem(index + 1, item)
		widgetList.setCurrentRow(index + 1)
			

	def RunRig( self ):
		
		OrientJoints(allJoints)
		OrientEndJoints(allJoints)
		
		squashAndStretch = 0
		enableTwist = 0
		heelPeelVar = 0
		toeTapVar = 0
		toeTipVar = 0
		ankleVar = 0
		fkList = FormList(allJoints, ui.listFK)
		ikList = FormList(allJoints, ui.listIK)
		splineList = FormList(allJoints, ui.listSpline)
		
		if (len(ikList) > 0):
			if (ui.heelPeel.isChecked()):
				heelPeelVar = 1
			if (ui.toeTap.isChecked()):
				toeTapVar = 1
			if (ui.toeTip.isChecked()):
				toeTipVar = 1
			if (ui.ankle.isChecked()):
				ankleVar = 1
			
			for i in range(len(ikList)):
				if (i % 3 == 0):
					ikParentings.append(SetIK(ikList[i], ikList[i + 1], ikList[i + 2], jointList, heelPeelVar, toeTapVar, toeTipVar, ankleVar))
		
		if (len(fkList) > 0):
			for i in range(len(fkList)):
				if (i % 2 == 0):
					fkParentings.append(SetFK(fkList[i], fkList[i + 1], jointList))
		
		if (ui.drivenKeys.isChecked()):
			drivenList = FormList(allJoints, ui.listDriven)
			if (len(drivenList) > 0):
				for jnt in drivenList:
					drivenParentings.append(RunDriven(jnt, allJoints))
					drivenParentings.append(fkParentings[0][1].getChildren()[0])
					drivenParentings.append(fkParentings[1][1].getChildren()[0])
				#else:
					#drivenParentings.append(fkParentings[1][1].getChildren()[0])
					
		
		if (len(splineList) > 0):
			if (ui.squashNStretch.isChecked()):
				squashAndStretch = 1
			if (ui.twistControls.isChecked()):
				enableTwist = 1
				
			splineParentings = (CreateIkSpline(splineList, rotationOrder, enableTwist, squashAndStretch))
		completeHierarchy(splineParentings, fkParentings, ikParentings, drivenParentings)


			
######################################Main############################################
"""Loading UI"""
ui = loadUI('C:/Users/Sinlord/Desktop/mainwindow.ui')
cont = UIController(ui)