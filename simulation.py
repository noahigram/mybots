from world import WORLD
from robot import ROBOT
import pybullet as p
import pybullet_data
import constants as c
import pyrosim.pyrosim as pyrosim
import time as t


class SIMULATION:
    def __init__(self, directOrGUI):
        self.directOrGUI = directOrGUI

        if directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
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
            # print(self.step)
            p.stepSimulation()
            self.robot.Sense(self.step)
            self.robot.Think()
            self.robot.Act(self.robot, self.step)
            self.step += 1
            if self.directOrGUI == "GUI":
                t.sleep(c.sleepInt)

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        p.disconnect()
