from world import WORLD
from robot import ROBOT
import pybullet as p
import pybullet_data
import constants as c
import pyrosim.pyrosim as pyrosim
import time as t


class SIMULATION:
    def __init__(self):

        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(c.gravX, c.gravY, c.gravZ)
        self.world = WORLD()
        self.robot = ROBOT()
        pyrosim.Prepare_To_Simulate(self.robot.robotID)
        self.step = 0

    def Run(self):
        self.step = 0
        while self.step < c.simLength:
            print(self.step)
            p.stepSimulation()
            self.robot.Sense(self.step)
            self.step += 1

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

            t.sleep(c.sleepInt)

    def __del__(self):
        p.disconnect()
