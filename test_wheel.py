import numpy as np
import pybullet as p
import pybullet_data
import time

# set up client with GUI
client = p.connect(p.GUI)

# set up data paths for pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# set gravity and ground
# p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])

# load parallelogram bot urdf
start_pos = [0, 0, 1]
start_orientation = p.getQuaternionFromEuler([0., 0., 0.])
urdf = "wheel_with_motor.urdf"
botId = p.loadURDF(urdf, start_pos, start_orientation, globalScaling=0.01)

for i in range(p.getNumJoints(botId)):
  print(p.getJointInfo(botId, i))

# set up wheel sliders
targetVelocitySlider = p.addUserDebugParameter("targetVelocity1", -10, 10, 0)

# run simulation
while True:
    # set wheel velocities
    p.setJointMotorControlArray(bodyIndex=botId, 
                                jointIndices=[0], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [p.readUserDebugParameter(targetVelocitySlider)])

    # step
    p.stepSimulation()
    
    # time.sleep(1./240.)