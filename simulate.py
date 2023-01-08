import pybullet as p
import pybullet_data
import time as t

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeID = p.loadURDF("plane.urdf")
p.loadSDF("boxes.sdf")
for ii in range(1000):
    p.stepSimulation()
    print(ii)
    t.sleep(1/60)

p.disconnect()
