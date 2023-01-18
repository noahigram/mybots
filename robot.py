import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR


class ROBOT:
    def __init__(self):
        self.robotID = p.loadURDF("body.urdf")
        self.motors = {}
        self.linkname = ""
        self.sensors = {}
        self.jointName = ""
        pyrosim.Prepare_To_Simulate(self.robotID)
        self.Prepare_To_Sense()

    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)
            self.linkName = linkName

    def Sense(self, t):
        for sensor in self.sensors.values():
            sensor.Get_Value(t)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.sensors[jointName] = SENSOR(jointName)
            self.jointName = jointName
