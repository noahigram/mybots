import pybullet as p
import pyrosim.pyrosim as pyrosim
from sensor import SENSOR
from motor import MOTOR
import constants as c
import numpy as np
from pyrosim.neuralNetwork import NEURAL_NETWORK
import os


class ROBOT:
    def __init__(self, solutionID):
        self.solutionID = solutionID
        self.robotID = p.loadURDF("body.urdf")
        self.motors = {}
        self.linkname = ""
        self.sensors = {}
        self.jointName = ""
        pyrosim.Prepare_To_Simulate(self.robotID)
        self.Prepare_To_Sense()
        self.Prepare_To_Act()
        self.nn = NEURAL_NETWORK(f"brain{self.solutionID}.nndf")
        os.system(f"rm brain{solutionID}.nndf")

    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)
            self.linkName = linkName

    def Sense(self, t):
        for sensor in self.sensors.values():
            sensor.Get_Value(t)

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)
            self.jointName = jointName

    def Act(self, robot, t):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desiredAngle = self.nn.Get_Value_Of(
                    neuronName) * c.motorJointRange
                self.motors[jointName].Set_Value(robot, desiredAngle)

    def Think(self):
        self.nn.Update()
        # self.nn.Print()

    def Get_Fitness(self, solutionID):
        #stateOfLinkZero = p.getLinkState(self.robotID, 0)
        basePositionAndOrientation = p.getBasePositionAndOrientation(
            self.robotID)
        #positionOfLinkZero = stateOfLinkZero[0]
        basePosition = basePositionAndOrientation[0]
        #xCoordinateOfLinkZero = positionOfLinkZero[0]
        xCoordinateOfLinkZero = basePosition[0]
        f = open(f"tmp{solutionID}.txt", "w")
        os.system(f"mv tmp{solutionID}.txt fitness{solutionID}.txt")
        f.write(str(xCoordinateOfLinkZero))
