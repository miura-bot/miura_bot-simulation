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

for i in range(p.getNumJoints(botId)):
    if p.getJointInfo(botId, i)[1] == b'base_link_b_joint':
        link_joint = i
        break

targetPos = p.addUserDebugParameter("targetPos", -3.14, 3.14, 0)

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

    # step
    p.stepSimulation()
    time.sleep(1./240.)