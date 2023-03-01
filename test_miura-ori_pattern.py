import numpy as np
import pybullet as p
import pybullet_data
import time

# set up client with GUI
client = p.connect(p.GUI)

# set up data paths for pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# set gravity and ground
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# load parallelogram bot urdf
start_pos = [0, 0, 1]
start_orientation = p.getQuaternionFromEuler([0., 0., 0.])
urdf = "miura-ori_pattern.urdf"
# urdf = "miura-ori_pattern-2cells.urdf"
botId = p.loadURDF(urdf, start_pos, start_orientation, globalScaling=0.01)

wheel_a_x = []
wheel_a_y = []
wheel_b_x = []
wheel_b_y = []
wheel_c_x = []
wheel_c_y = []
wheel_d_x = []
wheel_d_y = []

nJoints = p.getNumJoints(botId)
jointNameToId = {}
for i in range(nJoints):
    jointInfo = p.getJointInfo(botId, i)
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

for i in range(p.getNumJoints(botId)):
    # print(p.getJointInfo(botId, i)[1])
    if p.getJointInfo(botId, i)[1] == b'shaft_a_b':
        a_b_joint = i
    if p.getJointInfo(botId, i)[1] == b'shaft_b_c':
        b_c_joint = i
    if p.getJointInfo(botId, i)[1] == b'shaft_c_d':
        c_d_joint = i
    elif p.getJointInfo(botId, i)[1] == b'shaft_1_a' or p.getJointInfo(botId, i)[1] == b'shaft_3_a':
        wheel_a_x.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_2_a' or p.getJointInfo(botId, i)[1] == b'shaft_4_a':
        wheel_a_y.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_1_b' or p.getJointInfo(botId, i)[1] == b'shaft_3_b':
        wheel_b_x.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_2_b' or p.getJointInfo(botId, i)[1] == b'shaft_4_b':
        wheel_b_y.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_1_c' or p.getJointInfo(botId, i)[1] == b'shaft_3_c':
        wheel_c_x.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_2_c' or p.getJointInfo(botId, i)[1] == b'shaft_4_c':
        wheel_c_y.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_1_d' or p.getJointInfo(botId, i)[1] == b'shaft_3_d':
        wheel_d_x.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_2_d' or p.getJointInfo(botId, i)[1] == b'shaft_4_d':
        wheel_d_y.append(i)

targetPosAB = p.addUserDebugParameter("targetPosAB", -3.14, 3.14, 0)
# targetPosBC = p.addUserDebugParameter("targetPosBC", -3.14, 3.14, 0)
# targetPosCD = p.addUserDebugParameter("targetPosCD", -3.14, 3.14, 0)

targetVelocitySlider1a = p.addUserDebugParameter("targetVelocity1a", -2.0, 2.0, 0)
targetVelocitySlider2a = p.addUserDebugParameter("targetVelocity2a", -2.0, 2.0, 0)

targetVelocitySlider1b = p.addUserDebugParameter("targetVelocity1b", -2.0, 2.0, 0)
targetVelocitySlider2b = p.addUserDebugParameter("targetVelocity2b", -2.0, 2.0, 0)

targetVelocitySlider1c = p.addUserDebugParameter("targetVelocity1c", -2.0, 2.0, 0)
targetVelocitySlider2c = p.addUserDebugParameter("targetVelocity2c", -2.0, 2.0, 0)

targetVelocitySlider1d = p.addUserDebugParameter("targetVelocity1d", -2.0, 2.0, 0)
targetVelocitySlider2d = p.addUserDebugParameter("targetVelocity2d", -2.0, 2.0, 0)

testWheel = p.addUserDebugParameter("testWheel", -2.0, 2.0, 0)

joint_constraint = p.createConstraint(
    botId, jointNameToId["shaft_a_cylinder"], 
    botId, jointNameToId["shaft_d_cylinder"], 
    p.JOINT_FIXED, [0, 0, 0], [0, 80, 0], [0, 80, 0])

p.changeConstraint(joint_constraint, maxForce=10.0)

# run simulation
while True:
    p.setJointMotorControl2(
        bodyIndex=botId, 
        jointIndex=a_b_joint, 
        controlMode=p.POSITION_CONTROL, 
        targetPosition = p.readUserDebugParameter(targetPosAB),
        positionGain=1./12.,
        velocityGain=0.4,
    )

    # p.setJointMotorControl2(
    #     bodyIndex=botId, 
    #     jointIndex=b_c_joint, 
    #     controlMode=p.POSITION_CONTROL, 
    #     targetPosition = -2.0 * p.readUserDebugParameter(targetPosAB), # targetPosBC
    #     positionGain=1./12.,
    #     velocityGain=0.4,
    # )

    p.setJointMotorControl2(
        bodyIndex=botId, 
        jointIndex=c_d_joint, 
        controlMode=p.POSITION_CONTROL, 
        targetPosition = p.readUserDebugParameter(targetPosAB), # targetPosCD
        positionGain=1./12.,
        velocityGain=0.4,
    )

    targetVelocity1a = p.readUserDebugParameter(targetVelocitySlider1a)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_a_x, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity1a, targetVelocity1a])
    targetVelocity2a = p.readUserDebugParameter(targetVelocitySlider2a)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_a_y, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity2a, targetVelocity2a])

    targetVelocity1b = p.readUserDebugParameter(targetVelocitySlider1b)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_b_x, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity1b, targetVelocity1b])
    targetVelocity2b = p.readUserDebugParameter(targetVelocitySlider2b)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_b_y, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity2b, targetVelocity2b])
    
    targetVelocity1c = p.readUserDebugParameter(targetVelocitySlider1c)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_c_x, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity1c, targetVelocity1c])
    targetVelocity2c = p.readUserDebugParameter(targetVelocitySlider2c)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_c_y, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity2c, targetVelocity2c])
    
    targetVelocity1d = p.readUserDebugParameter(targetVelocitySlider1d)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_d_x, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity1d, targetVelocity1d])
    targetVelocity2d = p.readUserDebugParameter(targetVelocitySlider2d)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_d_y, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity2d, targetVelocity2d])

    # step
    p.stepSimulation()
    time.sleep(1./240.)