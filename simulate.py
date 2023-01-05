import pybullet as p
import time as t
physicsClient = p.connect(p.GUI)
for ii in range(1000):
    p.stepSimulation()
    print(ii)
    t.sleep(1/240)

p.disconnect()
