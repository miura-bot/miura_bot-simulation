import pybullet as p

client = p.connect(p.GUI)
start_pos = [0, 0, 0.4]
start_orientation = p.getQuaternionFromEuler([0., 0., 0.])
urdf = "miura_bot.urdf"
botId = p.loadURDF(urdf, start_pos, start_orientation)

while True:
    p.stepSimulation()