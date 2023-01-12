import pybullet as p
import pybullet_data
import time as t
import pyrosim.pyrosim as pyrosim
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeID = p.loadURDF("plane.urdf")
robotID = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotID)
simLength = 300
backLegSensorValues = np.zeros(simLength)
frontLegSensorValues = np.zeros(simLength)

for ii in range(simLength):
    p.stepSimulation()
    backLegSensorValues[ii] = pyrosim.Get_Touch_Sensor_Value_For_Link(
        "BackLeg")
    frontLegSensorValues[ii] = pyrosim.Get_Touch_Sensor_Value_For_Link(
        "FrontLeg")

    t.sleep(1/60)

np.save("data/backLegSensorValues.npy", backLegSensorValues)
np.save("data/frontLegSensorValues.npy", frontLegSensorValues)
p.disconnect()
