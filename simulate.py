from simulation import SIMULATION
import sys

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]

simulation = SIMULATION(directOrGUI, solutionID)
simulation.Run()

simulation.Get_Fitness()
# import pybullet as p
# import pybullet_data
# import time as t
# import pyrosim.pyrosim as pyrosim
# import numpy as np
# import random as rnd
# import constants as c


# # Store sine values in the range (-1 to 1)
# x = np.linspace(0, 2*np.pi, c.simLength)


# np.save("data/targetAnglesValuesFront.npy", targetAnglesFront)
# np.save("data/targetAnglesValuesBack.npy", targetAnglesBack)

# for ii in range(c.simLength):
#     p.stepSimulation()
#     backLegSensorValues[ii] = pyrosim.Get_Touch_Sensor_Value_For_Link(
#         "BackLeg")
#     frontLegSensorValues[ii] = pyrosim.Get_Touch_Sensor_Value_For_Link(
#         "FrontLeg")

#     pyrosim.Set_Motor_For_Joint(
#         bodyIndex=robotID,
#         jointName=b'Torso_BackLeg',
#         controlMode=p.POSITION_CONTROL,
#         targetPosition=targetAnglesBack[ii],
#         maxForce=c.backLegForce
#     )
#     pyrosim.Set_Motor_For_Joint(
#         bodyIndex=robotID,
#         jointName=b'Torso_FrontLeg',
#         controlMode=p.POSITION_CONTROL,
#         targetPosition=targetAnglesFront[ii],
#         maxForce=c.frontLegForce
#     )

#     t.sleep(c.SleepInt)


# np.save("data/backLegSensorValues.npy", backLegSensorValues)
# np.save("data/frontLegSensorValues.npy", frontLegSensorValues)
#
