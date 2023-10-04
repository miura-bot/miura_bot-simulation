import numpy as np
import pybullet as p
import pybullet_data
import time
from scipy.spatial.transform import Rotation

# set up client with GUI
client = p.connect(p.GUI)

# set up data paths for pybullet_data
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# set gravity and ground
p.setGravity(0, 0, -9.81)
p.loadURDF("plane.urdf", [0, 0, 0], [0, 0, 0, 1])
p.setTimeStep(0.008)

# set target position and orientation relative to the first parallelogram
target_pos = [3.0, 0, 0]
target_orientation = p.getQuaternionFromEuler([0.0, 0., np.pi/3])
target_orientation = p.getEulerFromQuaternion(target_orientation)

final_target_pos = [1.35, -0.3, 0]

# load parallelogram bot urdf
start_pos_0 = [0, 0, 0.2]
start_orientation_0 = p.getQuaternionFromEuler([0., 0., 0.])
urdf = "miura_bot.urdf"
bot0Id = p.loadURDF(urdf, start_pos_0, start_orientation_0, globalScaling=0.01)

start_pos_1 = [np.random.uniform(5, 7), np.random.uniform(-5, 5), 0.2]
start_orientation_1 = p.getQuaternionFromEuler([0., 0., np.random.uniform(0, 2 * np.pi)])
bot1Id = p.loadURDF(urdf, start_pos_1, start_orientation_1, globalScaling=0.01)

vector_done = False
rotate_done = False

v1_vector = np.array([1.0, 0.0, 0.0]).reshape(3, 1)
v2_vector = np.array([-0.5, np.sqrt(3)/2.0, 0.0]).reshape(3, 1)

while True:
    pos0, ang0 = p.getBasePositionAndOrientation(bot0Id, physicsClientId = client)
    ang0_euler = p.getEulerFromQuaternion(ang0)
    pos1, ang1 = p.getBasePositionAndOrientation(bot1Id, physicsClientId = client)
    ang1_euler = p.getEulerFromQuaternion(ang1)
        
    if not vector_done and not np.isclose(np.add(pos0, target_pos), pos1, atol=0.1).all():
        p.addUserDebugLine(np.add(pos0, target_pos), pos1, lifeTime=3.0, lineColorRGB=[1, 0, 0])

        vector_to_follow = np.subtract(np.add(pos0, target_pos), pos1)
        rotmat = Rotation.from_quat(ang1).as_matrix()
        
        robot_coor = rotmat@np.eye(3)
        s1 = vector_to_follow@(robot_coor[:, 0]*2.0)
        s2 = vector_to_follow@(robot_coor[:, 1]*2.0)
        print("translate", s1, s2)

        p.addUserDebugLine([0.0, 0.0, 0.0], v1_vector * s1, lifeTime=3, lineColorRGB=[0, 0, 1], parentObjectUniqueId=bot1Id)
        p.addUserDebugLine([0.0, 0.0, 0.0], v2_vector * s2, lifeTime=3, lineColorRGB=[0, 1, 0], parentObjectUniqueId=bot1Id)

        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                    jointIndices=[0, 2], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [s1, s1])
        
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                    jointIndices=[1, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [-s2, -s2])
        
    elif not rotate_done and not np.isclose(np.array(ang1_euler)[2], np.add(ang0_euler, target_orientation)[2], atol=np.radians(3)).all():
        vector_done = True
        print("rotate", np.array(ang1_euler)[2], np.add(ang0_euler, target_orientation)[2])
        if (np.array(ang1)[2] > np.add(ang0, target_orientation)[2]):
            targetVelocity = -10 * abs(np.array(ang1)[2] - np.add(ang0, target_orientation)[2])
        else:
            targetVelocity = 10 * abs(np.array(ang1)[2] - np.add(ang0, target_orientation)[2])
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[1, 3], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity, -targetVelocity])
        
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[0, 2], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [0, 0])
        
    elif not np.isclose(np.array(pos1)[0], np.add(pos0, final_target_pos)[0], atol=0.1).all():
        rotate_done = True
        if np.array(pos1)[0] > np.add(pos0, final_target_pos)[0]:
            targetVelocity = -5
        else:
            targetVelocity = 5
        print("move left right", np.array(pos1)[0], np.add(pos0, final_target_pos)[0], targetVelocity)
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[1, 3], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity, targetVelocity])
        
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[0, 2], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [0, 0])

    else:
        print("done", pos0, ang0, pos1, ang1)
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[0, 1, 2, 3], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [0, 0, 0, 0])

    p.stepSimulation()
    time.sleep(0.001)