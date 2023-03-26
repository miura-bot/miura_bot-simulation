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

# set target position and orientation relative to the first parallelogram
target_pos = [3.0, 0, 0]
# target_pos = [1.4, 0, 0]
target_orientation = p.getQuaternionFromEuler([0., 0., np.pi/3])
target_orientation = p.getEulerFromQuaternion(target_orientation)

# load parallelogram bot urdf
start_pos_0 = [0, 0, 0.2]
start_orientation_0 = p.getQuaternionFromEuler([0., 0., 0.])
urdf = "miura_bot.urdf"
bot0Id = p.loadURDF(urdf, start_pos_0, start_orientation_0, globalScaling=0.01)

# start_pos_1 = [np.random.uniform(5, 7), np.random.uniform(-5, 5), 0.2]
start_pos_1 = [5, 3, 0.2]
# start_orientation_1 = p.getQuaternionFromEuler([0., 0., np.random.uniform(0, np.pi)])
start_orientation_1 = p.getQuaternionFromEuler([0., 0., 0])
bot1Id = p.loadURDF(urdf, start_pos_1, start_orientation_1, globalScaling=0.01)

vector_done = False

while True:

    pos0, ang0 = p.getBasePositionAndOrientation(bot0Id, physicsClientId = client)
    ang0 = p.getEulerFromQuaternion(ang0)
    pos1, ang1 = p.getBasePositionAndOrientation(bot1Id, physicsClientId = client)
    ang1 = p.getEulerFromQuaternion(ang1)
        
    if not vector_done and not np.isclose(np.add(pos0, target_pos), pos1, atol=0.1).all():
    # if True:

        p.addUserDebugLine(np.add(pos0, target_pos), pos1, lifeTime=3, lineColorRGB=[1, 0, 0])

        vector_to_follow = np.subtract(np.add(pos0, target_pos), pos1)
        # s1 = vector_to_follow[1] - (np.sqrt(3)/3.) * vector_to_follow[0] * 10
        # s2 = 2.*np.sqrt(3)/3. * vector_to_follow[0] * 10
        s1 = vector_to_follow[1] + (np.sqrt(3)/3.) * vector_to_follow[0] * 10 * 1.5
        s2 = 2.*np.sqrt(3)/3. * vector_to_follow[0] * 10 * -1
        print("move", s1, s2)

        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                    jointIndices=[0, 2], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [-s2, -s2])
        
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                    jointIndices=[1, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [-s1, -s1])
        
    elif not np.isclose(np.array(ang1)[2], np.add(ang0, target_orientation)[2], atol=0.1).all():
        vector_done = True
        # print(np.array(ang1), np.add(ang0, target_orientation), np.isclose(np.array(ang1), np.add(ang0, target_orientation), atol=0.1).all())
        print("rotate")
        targetVelocity = 10
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[1, 3], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity, -targetVelocity])
        
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[0, 2], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [0, 0])
        
    elif not np.isclose(np.array(pos1)[0], np.add(pos0, target_pos)[0], atol=0.2).all():
        print("move left right")
        if np.array(pos1)[0] > np.add(pos0, target_pos)[0]:
            targetVelocity = -15
        else:
            targetVelocity = 15
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[1, 3], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [targetVelocity, targetVelocity])
        
        p.setJointMotorControlArray(bodyIndex=bot1Id, 
                                jointIndices=[0, 2], 
                                controlMode=p.VELOCITY_CONTROL, 
                                targetVelocities = [0, 0])

    # if not np.isclose(np.array(ang1), np.add(ang0, target_orientation), atol=0.1).all():
    #     # print(np.array(ang1), np.add(ang0, target_orientation), np.isclose(np.array(ang1), np.add(ang0, target_orientation), atol=0.1).all())
    #     print("rotate")
    #     targetVelocity = 30
    #     p.setJointMotorControlArray(bodyIndex=bot1Id, 
    #                             jointIndices=[1, 3], 
    #                             controlMode=p.VELOCITY_CONTROL, 
    #                             targetVelocities = [targetVelocity, -targetVelocity])
        
    #     p.setJointMotorControlArray(bodyIndex=bot1Id, 
    #                             jointIndices=[0, 2], 
    #                             controlMode=p.VELOCITY_CONTROL, 
    #                             targetVelocities = [0, 0])
        
    # elif not np.isclose(np.array(pos1)[1], np.add(pos0, target_pos)[1], atol=0.1).all():
    #     # print(np.array(pos1), np.add(pos0, target_pos), np.isclose(np.array(pos1), np.add(pos0, target_pos), atol=0.1).all())
    #     print("move at angle")
    #     if np.array(pos1)[1] > np.add(pos0, target_pos)[1]:
    #         targetVelocity = 20
    #     else:
    #         targetVelocity = -20
    #     p.setJointMotorControlArray(bodyIndex=bot1Id, 
    #                             jointIndices=[0, 2], 
    #                             controlMode=p.VELOCITY_CONTROL, 
    #                             targetVelocities = [targetVelocity, targetVelocity])
        
    #     p.setJointMotorControlArray(bodyIndex=bot1Id, 
    #                             jointIndices=[1, 3], 
    #                             controlMode=p.VELOCITY_CONTROL, 
    #                             targetVelocities = [0, 0])
    
    # elif not np.isclose(np.array(pos1)[0], np.add(pos0, target_pos)[0], atol=0.1).all():
    #     # print(np.array(pos1), np.add(pos0, target_pos), np.isclose(np.array(pos1), np.add(pos0, target_pos), atol=0.1).all())
    #     print("move left right")
    #     if np.array(pos1)[0] > np.add(pos0, target_pos)[0]:
    #         targetVelocity = 20
    #     else:
    #         targetVelocity = -20
    #     p.setJointMotorControlArray(bodyIndex=bot1Id, 
    #                             jointIndices=[1, 3], 
    #                             controlMode=p.VELOCITY_CONTROL, 
    #                             targetVelocities = [targetVelocity, targetVelocity])
        
    #     p.setJointMotorControlArray(bodyIndex=bot1Id, 
    #                             jointIndices=[0, 2], 
    #                             controlMode=p.VELOCITY_CONTROL, 
    #                             targetVelocities = [0, 0])
    # else:
    #     targetVelocity = 0
    #     p.setJointMotorControlArray(bodyIndex=bot1Id, 
    #                             jointIndices=[0, 1, 2, 3], 
    #                             controlMode=p.VELOCITY_CONTROL, 
    #                             targetVelocities = [targetVelocity, targetVelocity, targetVelocity, targetVelocity])

    #     print("done")

    p.stepSimulation()