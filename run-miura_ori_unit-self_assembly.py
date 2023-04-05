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

parallelogram_module_urdf = "miura_bot.urdf"

##### TARGET POSITIONS AND ORIENTATIONS #####

# stage 0 (starting) positions and orientations
# TODO: Add random starting configurations within the quadrant
# TODO: (future) Assign ids based on starting configurations

min_x = 2
max_x = 6

bot1_starting_pos = [np.random.uniform(min_x, max_x), np.random.uniform(min_x, max_x), 0.2]
bot1_orientation = p.getQuaternionFromEuler([0., 0., np.random.uniform(0, 2*np.pi)])

bot2_starting_pos = [-np.random.uniform(min_x, max_x), np.random.uniform(min_x, max_x), 0.2]
bot2_orientation = p.getQuaternionFromEuler([0., 0., np.random.uniform(0, 2*np.pi)])

bot3_starting_pos = [-np.random.uniform(min_x, max_x), -np.random.uniform(min_x, max_x), 0.2]
bot3_orientation = p.getQuaternionFromEuler([0., 0., np.random.uniform(0, 2*np.pi)])

bot4_starting_pos = [np.random.uniform(min_x, max_x), -np.random.uniform(min_x, max_x), 0.2]
bot4_orientation = p.getQuaternionFromEuler([0., 0., np.random.uniform(0, 2*np.pi)])

# stage 1 target positions and orientations

bot4_stage1_target_pos_relative_bot3 = [3.0, 0., 0.]
bot4_stage1_target_orientation_relative_bot3 = [0., 0., np.pi/3]

# stage 2 target positions and orientations

bot1_stage2_target_pos_relative_bot4 = np.add([-3*np.cos(np.radians(120)), 3*np.sin(np.radians(120)), 0.], [0, 1.65, 0.2])
bot1_stage2_target_orientation_relative_bot4 = [0., 0., 0.]
bot1_stage2_1_target_pos_relative_bot4 = np.add([-1.35*np.cos(np.radians(120)), 1.35*np.sin(np.radians(120)), 0.], [0, 1.65, 0.2])

bot2_stage2_target_pos_relative_bot3 = np.add([-3*np.cos(np.radians(60)), 3*np.sin(np.radians(60)), 0.], [0, 1.65, 0.2])
bot2_stage2_target_orientation_relative_bot3 = [0., 0., 0.]
bot2_stage2_1_target_pos_relative_bot3 = np.add([-1.35*np.cos(np.radians(60)), 1.35*np.sin(np.radians(60)), 0.], [0, 1.65, 0.2])

# stage 3 target positions and orientations

bot1_stage3_target_pos_relative_bot2 = [1.35, 0., 0.]
bot1_stage3_target_orientation_relative_bot2 = p.getQuaternionFromEuler([0., 0., 0.])

bot4_stage3_target_pos_relative_bot3 = [1.35, 0., 0.]
bot4_stage3_target_orientation_relative_bot3 = p.getQuaternionFromEuler([0., 0., 0.])

##### MOVEMENT HELPER FUNCTIONS #####

# TODO: FIX SCALING
def vector_translation(botA_id, botB_id, botA_target_pos_relative_botB, atol=0.1):
    
    botA_pos, botA_orientation = p.getBasePositionAndOrientation(botA_id, physicsClientId=client)
    botA_orientation = p.getEulerFromQuaternion(botA_orientation)
    botB_pos, botB_orientation = p.getBasePositionAndOrientation(botB_id, physicsClientId=client)
    botB_orientation = p.getEulerFromQuaternion(botB_orientation)
    if not np.isclose(np.add(botB_pos, botA_target_pos_relative_botB), botA_pos, atol=atol).all():
        
        p.addUserDebugLine(np.add(botB_pos, botA_target_pos_relative_botB), botA_pos, lifeTime=3, lineColorRGB=[1, 0, 0])

        vector = np.subtract(np.add(botB_pos, botA_target_pos_relative_botB), botA_pos)

        vector_rotation_angle = -botA_orientation[2]
        vector_rotation_matrix = np.array([[np.cos(vector_rotation_angle), -np.sin(vector_rotation_angle), 0],
                                [np.sin(vector_rotation_angle), np.cos(vector_rotation_angle), 0],
                                [0, 0, 1]])
        vector = np.dot(vector_rotation_matrix, vector)


        s1 = vector[1] + (np.sqrt(3)/3.) * vector[0] * 10 * 1.75
        s2 = 2.*np.sqrt(3)/3. * vector[0] * 10 * -1
        if abs(s1) < 1 or abs(s2) < 1:
            s1 *= 5
            s2 *= 5
        print("vector_translation", s1, s2)

        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[0, 2], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [-s2, -s2])
        
        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[1, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [-s1, -s1])
        
        return False
    else:
        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[0, 1, 2, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [0, 0, 0, 0])
        return True
    
# TODO: REMOVE WHEN vector_translation WORKS FOR ALL ORIENTATIONS
def vector_translation_temp(botA_id, botB_id, botA_target_pos_relative_botB, atol=0.5, targetVelocities=[0, 0, 0, 0]):
    
    botA_pos, botA_orientation = p.getBasePositionAndOrientation(botA_id, physicsClientId=client)
    botA_orientation = p.getEulerFromQuaternion(botA_orientation)
    botB_pos, botB_orientation = p.getBasePositionAndOrientation(botB_id, physicsClientId=client)
    botB_orientation = p.getEulerFromQuaternion(botB_orientation)
    if not np.isclose(np.add(botB_pos, botA_target_pos_relative_botB), botA_pos, atol=atol).all():

        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[0, 1, 2, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = targetVelocities)
        
        return False
    else:
        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[0, 1, 2, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [0, 0, 0, 0])
        return True
    
def rotation(botA_id, botB_id, botA_target_orientation_relative_botB, atol=np.radians(1)):
    
    botA_pos, botA_orientation = p.getBasePositionAndOrientation(botA_id, physicsClientId=client)
    botA_orientation = p.getEulerFromQuaternion(botA_orientation)
    if botB_id is not None:
        botB_pos, botB_orientation = p.getBasePositionAndOrientation(botB_id, physicsClientId=client)
        botB_orientation = p.getEulerFromQuaternion(botB_orientation)
    else:
        botB_orientation = [0, 0, 0]
    if not np.isclose(np.add(botB_orientation, botA_target_orientation_relative_botB), botA_orientation, atol=atol).all():

        yaw_diff = np.subtract(np.add(botB_orientation, botA_target_orientation_relative_botB), botA_orientation)[2]
        min_abs_velocity = 10
        if (yaw_diff < 0):
            targetVelocity = min(-10 * abs(yaw_diff), min_abs_velocity)
        else:
            targetVelocity = max(10 * abs(yaw_diff), min_abs_velocity)

        print("rotation", botA_orientation, np.add(botB_orientation, botA_target_orientation_relative_botB), targetVelocity)

        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[1, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [targetVelocity, -targetVelocity])
        
        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[0, 2], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [0, 0])
        
        return False
    else:
        p.setJointMotorControlArray(bodyIndex=botA_id, 
                                    jointIndices=[0, 1, 2, 3], 
                                    controlMode=p.VELOCITY_CONTROL, 
                                    targetVelocities = [0, 0, 0, 0])
        return True

##### LOAD URDF FILES #####

# Load 4 parallelogram modules from URDF into 4 quadrants
bot1_id = p.loadURDF(parallelogram_module_urdf, bot1_starting_pos, bot1_orientation, globalScaling=0.01)
bot2_id = p.loadURDF(parallelogram_module_urdf, bot2_starting_pos, bot2_orientation, globalScaling=0.01)
bot3_id = p.loadURDF(parallelogram_module_urdf, bot3_starting_pos, bot3_orientation, globalScaling=0.01)
bot4_id = p.loadURDF(parallelogram_module_urdf, bot4_starting_pos, bot4_orientation, globalScaling=0.01)

##### STEP SIMUlATION FOR SELF-ASSEMBLY #####

stage = 0
vector_translation_complete_1a = False
rotation_complete_1a = False
vector_rotation_complete_1b = False
vector_translation_complete_2a = False
rotation_complete_2a = False
vector_translation_complete_2a1 = False
vector_translation_complete_2b = False
rotation_complete_2b = False
vector_translation_complete_2b1 = False
vector_translation_complete_3a = False
while True:
    if stage == 0:
        print("Stage %d: alligning quadrant 4 parallelogram to quadrant 3 parallelogram" % stage)

        if not vector_translation_complete_1a:
            vector_translation_complete_1a = vector_translation(bot4_id, bot3_id, bot4_stage1_target_pos_relative_bot3, atol=0.05)
        elif not rotation_complete_1a or not vector_rotation_complete_1b:
            # TODO: change this to be relative to bot3 rather than absolute to the world
            rotation_complete_1a = rotation(bot4_id, None, bot4_stage1_target_orientation_relative_bot3)

        if not vector_rotation_complete_1b:
            # TODO: change this to be relative to bot4 rather than absolute to the world
            vector_rotation_complete_1b = rotation(bot3_id, None, [0, 0, 0])
        
        if vector_translation_complete_1a and rotation_complete_1a and vector_rotation_complete_1b:
            stage = 1
    elif stage == 1:
        print("Stage %d: alligning top quadrant parallelograms with bottom quadrant parallelograms" % stage)
        # TODO: parallelize this with the previous step

        if not vector_translation_complete_2a:
            vector_translation_complete_2a = vector_translation(bot1_id, bot4_id, bot1_stage2_target_pos_relative_bot4)
        elif not rotation_complete_2a:
            rotation_complete_2a = rotation(bot1_id, bot4_id, bot1_stage2_target_orientation_relative_bot4)
        elif not vector_translation_complete_2a1:
            vector_translation_complete_2a1 = vector_translation(bot1_id, bot4_id, bot1_stage2_1_target_pos_relative_bot4)
            # vector_translation_complete_2a1 = vector_translation_temp(bot1_id, bot4_id, bot1_stage2_1_target_pos_relative_bot4, targetVelocities=[-5, 0, -5, 0])

        if not vector_translation_complete_2b:
            vector_translation_complete_2b = vector_translation(bot2_id, bot3_id, bot2_stage2_target_pos_relative_bot3)
        elif not rotation_complete_2b:
            rotation_complete_2b = rotation(bot2_id, bot3_id, bot2_stage2_target_orientation_relative_bot3)
        elif not vector_translation_complete_2b1:
            vector_translation_complete_2b1 = vector_translation(bot2_id, bot3_id, bot2_stage2_1_target_pos_relative_bot3)
            # vector_translation_complete_2b1 = vector_translation_temp(bot2_id, bot3_id, bot2_stage2_1_target_pos_relative_bot3, targetVelocities=[0, 5, 0, 5])

        if vector_translation_complete_2a and rotation_complete_2a and vector_translation_complete_2b and rotation_complete_2b and vector_translation_complete_2a1 and vector_translation_complete_2b1:
            stage = 2
    elif stage == 2:
        print("Stage %d: aligning right quadrant parallelograms (coupled) with left quadrant parallelograms (coupled)" % stage)

        if not vector_translation_complete_3a:
            vector_translation_complete_3a_top = vector_translation(bot1_id, bot2_id, bot1_stage3_target_pos_relative_bot2)
            vector_translation_complete_3a_bottom = vector_translation(bot4_id, bot3_id, bot4_stage3_target_pos_relative_bot3)
            # vector_translation_complete_3a_top = vector_translation_temp(bot1_id, bot2_id, bot1_stage3_target_pos_relative_bot2, targetVelocities=[0, 5, 0, 5])
            # vector_translation_complete_3a_bottom = vector_translation_temp(bot4_id, bot3_id, bot4_stage3_target_pos_relative_bot3, targetVelocities=[0, 5, 0, 5])

        if vector_translation_complete_3a_top and vector_translation_complete_3a_bottom:
            stage = 3
    elif stage == 3:
        print("Stage %d: Self-Assembly Complete" % stage)
    else:
        print("Stage Error: Stage %d is an invalid number." % stage)
    p.stepSimulation()