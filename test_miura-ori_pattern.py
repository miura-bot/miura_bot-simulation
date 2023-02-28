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
# urdf = "miura-ori_pattern.urdf"
urdf = "miura-ori_pattern-2cells.urdf"
botId = p.loadURDF(urdf, start_pos, start_orientation, globalScaling=0.01)

wheel_a_x = []
wheel_a_y = []

new_wheel = None

for i in range(p.getNumJoints(botId)):
    # print(p.getJointInfo(botId, i)[1])
    if p.getJointInfo(botId, i)[1] == b'shaft_a_b':
        link_joint = i
    elif p.getJointInfo(botId, i)[1] == b'shaft_1_a' or p.getJointInfo(botId, i)[1] == b'shaft_3_a':
        wheel_a_x.append(i)
    elif p.getJointInfo(botId, i)[1] == b'shaft_2_a' or p.getJointInfo(botId, i)[1] == b'shaft_4_a':
        wheel_a_y.append(i)  
    elif p.getJointInfo(botId, i)[1] == b'shaft_2_b':
        new_wheel = i

targetPos = p.addUserDebugParameter("targetPos", -3.14, 3.14, 0)
targetVelocitySlider = p.addUserDebugParameter("targetVelocity1", -2.0, 2.0, 0)
targetVelocitySlider2 = p.addUserDebugParameter("targetVelocity2", -2.0, 2.0, 0)

targetVelocitySlider3 = p.addUserDebugParameter("targetVelocity3", -2.0, 2.0, 0)

# run simulation
while True:
    p.setJointMotorControl2(
        bodyIndex=botId, 
        jointIndex=link_joint, 
        controlMode=p.POSITION_CONTROL, 
        targetPosition = p.readUserDebugParameter(targetPos),
        positionGain=1./12.,
        velocityGain=0.4,
    )

    targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_a_x, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity, targetVelocity])
    targetVelocity2 = p.readUserDebugParameter(targetVelocitySlider2)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=wheel_a_y, 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity2, targetVelocity2])

    targetVelocity3 = p.readUserDebugParameter(targetVelocitySlider3)
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=[new_wheel], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity3])

    # step
    p.stepSimulation()
    time.sleep(1./240.)