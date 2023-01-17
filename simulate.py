import pybullet as p
import pybullet_data
import time as t
import pyrosim.pyrosim as pyrosim
import numpy as np
import random as rnd

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
planeID = p.loadURDF("plane.urdf")
robotID = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotID)
simLength = 1000
backLegSensorValues = np.zeros(simLength)
frontLegSensorValues = np.zeros(simLength)

# Amplitude, frequency, phase offset
amplitude_front = np.pi/3
frequency_front = 40
phaseOffset_front = np.pi/3
amplitude_back = np.pi/4
frequency_back = 40
phaseOffset_back = 0

# Store sine values in the range (-1 to 1)
x = np.linspace(0, 2*np.pi, simLength)

targetAngles_front = amplitude_front * \
    np.sin(frequency_front*x+phaseOffset_front)
targetAngles_back = amplitude_back*np.sin(frequency_back*x+phaseOffset_back)
np.save("data/targetAnglesValuesFront.npy", targetAngles_front)
np.save("data/targetAnglesValuesBack.npy", targetAngles_back)

for ii in range(simLength):
    p.stepSimulation()
    backLegSensorValues[ii] = pyrosim.Get_Touch_Sensor_Value_For_Link(
        "BackLeg")
    frontLegSensorValues[ii] = pyrosim.Get_Touch_Sensor_Value_For_Link(
        "FrontLeg")

    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robotID,
        jointName=b'Torso_BackLeg',
        controlMode=p.POSITION_CONTROL,
        targetPosition=targetAngles_back[ii],
        maxForce=30
    )
    pyrosim.Set_Motor_For_Joint(
        bodyIndex=robotID,
        jointName=b'Torso_FrontLeg',
        controlMode=p.POSITION_CONTROL,
        targetPosition=targetAngles_front[ii],
        maxForce=30
    )

    t.sleep(1/240)


np.save("data/backLegSensorValues.npy", backLegSensorValues)
np.save("data/frontLegSensorValues.npy", frontLegSensorValues)
p.disconnect()
