import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID
        self.weights = np.random.rand(c.numSensorNeurons, c.numMotorNeurons)
        self.weights = self.weights * 2 - 1

    def Start_Simulation(self, runString):
        os.system("python3 simulate.py " + runString +
                  " " + str(self.myID) + " 2&>1 &")
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

    def Wait_For_Simulation_To_End(self):

        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.5)

        fitnessFileName = f"fitness{self.myID}.txt"
        f = open(fitnessFileName, "r")

        self.fitness = float(f.read())
        f.close()
        os.system(f"rm fitness{self.myID}.txt")

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(
            name="Box0", pos=[-1, -1, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box1", pos=[-2, -0.5, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box2", pos=[-3, 1, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box3", pos=[-2, 1, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box4", pos=[-4, 0.5, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box5", pos=[-5, -1, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box6", pos=[-4, -1.5, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box7", pos=[-3, -1, 0.5], size=[0.5, 0.5, 1], mass=100)
        pyrosim.Send_Cube(
            name="Box8", pos=[-1, 1, 0.5], size=[0.5, 0.5, 1], mass=100)

        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        # Simple dog
        # Torso
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1], size=[1, 0.5, 0.2])

        # Head
        pyrosim.Send_Cube(name="Head", pos=[0, 0, 0], size=[0.3, 0.2, 0.2])
        pyrosim.Send_Joint(name="Torso_Head", parent="Torso", child="Head",
                           type="revolute", position=[-0.5, 0, 1], jointAxis="1 0 0")

        # Tail
        pyrosim.Send_Cube(name="Tail", pos=[0, 0, 0], size=[0.8, 0.1, 0.1])
        pyrosim.Send_Joint(name="Torso_Tail", parent="Torso", child="Tail",
                           type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")

        # Front Left Leg
        pyrosim.Send_Joint(name="Torso_FrontLeftLeg", parent="Torso", child="FrontLeftLeg",
                           type="revolute", position=[-0.5, -0.1, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="FrontLeftLeg",
                          pos=[0, -0.1, -0.5], size=[0.1, 0.1, 1])

        # Front Right leg
        pyrosim.Send_Joint(name="Torso_FrontRightLeg", parent="Torso", child="FrontRightLeg",
                           type="revolute", position=[-0.5, 0.1, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="FrontRightLeg",
                          pos=[0, 0.1, -0.5], size=[0.1, 0.1, 1])

        # Back Left Leg
        pyrosim.Send_Joint(name="Torso_BackLeftLeg", parent="Torso", child="BackLeftLeg",
                           type="revolute", position=[0.5, -0.1, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="BackLeftLeg",
                          pos=[0, -0.1, -0.5], size=[0.1, 0.1, 1])

        # Back Right Leg
        pyrosim.Send_Joint(name="Torso_BackRightLeg", parent="Torso", child="BackRightLeg",
                           type="revolute", position=[0.5, 0.1, 1], jointAxis="0 1 0")
        pyrosim.Send_Cube(name="BackRightLeg",
                          pos=[0, 0.1, -0.5], size=[0.1, 0.1, 1])

        # pyrosim.Send_Cube(name="FrontLeftLeg",pos=[])

        # Robot with 3 links and two joints (Torso, Backleg and frontleg)
        # pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1], size=[1, 1, 1])

        # pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso",
        #                    child="BackLeg", type="revolute", position=[0, -0.5, 1], jointAxis="1 0 0")
        # pyrosim.Send_Cube(name="BackLeg", pos=[0, -0.5, 0], size=[0.2, 1, 0.2])

        # pyrosim.Send_Joint(name="BackLeg_BackLowerLeg", parent="BackLeg",
        #                    child="BackLowerLeg", type="revolute", position=[0, -1, 0], jointAxis="1 0 0")
        # pyrosim.Send_Cube(name="BackLowerLeg", pos=[
        #                   0, 0, -0.5], size=[0.2, 0.2, 1])

        # pyrosim.Send_Cube(
        #     name="LeftLeg", pos=[-0.5, 0, 0], size=[1.0, 0.2, 0.2])
        # pyrosim.Send_Joint(name="Torso_LeftLeg", parent="Torso",
        #                    child="LeftLeg", type="revolute", position=[-0.5, 0, 1], jointAxis="0 1 0")

        # pyrosim.Send_Joint(name="LeftLeg_LeftLowerLeg", parent="LeftLeg",
        #                    child="LeftLowerLeg", type="revolute", position=[-1, 0, 0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="LeftLowerLeg", pos=[
        #                   0, 0, -0.5], size=[0.2, 0.2, 1])

        # pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso",
        #                    child="FrontLeg", type="revolute", position=[0, 0.5, 1], jointAxis="1 0 0")
        # pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0], size=[0.2, 1, 0.2])

        # pyrosim.Send_Joint(name="FrontLeg_FrontLowerLeg", parent="FrontLeg",
        #                    child="FrontLowerLeg", type="revolute", position=[0, 1, 0], jointAxis="1 0 0")
        # pyrosim.Send_Cube(name="FrontLowerLeg", pos=[
        #                   0, 0, -0.5], size=[0.2, 0.2, 1])

        # pyrosim.Send_Joint(name="Torso_RightLeg", parent="Torso",
        #                    child="RightLeg", type="revolute", position=[0.5, 0, 1], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0], size=[1, 0.2, 0.2])

        # pyrosim.Send_Joint(name="RightLeg_RightLowerLeg", parent="RightLeg",
        #                    child="RightLowerLeg", type="revolute", position=[1, 0, 0], jointAxis="0 1 0")
        # pyrosim.Send_Cube(name="RightLowerLeg", pos=[
        #                   0, 0, -0.5], size=[0.2, 0.2, 1])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        pyrosim.Send_Sensor_Neuron(name=1, linkName="FrontLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontRightLeg")
        pyrosim.Send_Sensor_Neuron(name=3, linkName="BackLeftLeg")
        pyrosim.Send_Sensor_Neuron(name=4, linkName="BackRightLeg")

        # pyrosim.Send_Sensor_Neuron(name=2, linkName="BackLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name=3, linkName="LeftLeg")
        # pyrosim.Send_Sensor_Neuron(name=4, linkName="LeftLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name=5, linkName="FrontLeg")
        # pyrosim.Send_Sensor_Neuron(name=6, linkName="FrontLowerLeg")
        # pyrosim.Send_Sensor_Neuron(name=7, linkName="RightLeg")
        # pyrosim.Send_Sensor_Neuron(name=8, linkName="RightLowerLeg")

        pyrosim.Send_Motor_Neuron(name=5, jointName="Torso_FrontLeftLeg")
        pyrosim.Send_Motor_Neuron(name=6, jointName="Torso_FrontRightLeg")
        pyrosim.Send_Motor_Neuron(name=7, jointName="Torso_BackLeftLeg")
        pyrosim.Send_Motor_Neuron(name=8, jointName="Torso_BackRightLeg")
        # pyrosim.Send_Motor_Neuron(name=10, jointName="BackLeg_BackLowerLeg")
        #pyrosim.Send_Motor_Neuron(name=11, jointname="Torso_LeftLeg")
        # pyrosim.Send_Motor_Neuron(name=12, jointName="LeftLeg_LeftLowerLeg")
        # pyrosim.Send_Motor_Neuron(name=13, jointName="Torso_FrontLeg")
        # pyrosim.Send_Motor_Neuron(name=14, jointName="FrontLeg_FrontLowerLeg")
        # pyrosim.Send_Motor_Neuron(name=15, jointName="Torso_RightLeg")
        # pyrosim.Send_Motor_Neuron(name=16, jointName="RightLeg_RightLowerLeg")
        # pyrosim.Send_Synapse(sourceNeuronName=0,
        #                      targetNeuronName=3, weight=0.1)
        # pyrosim.Send_Synapse(sourceNeuronName=1,
        #                      targetNeuronName=3, weight=0.1)
        # pyrosim.Send_Synapse(sourceNeuronName=2,
        #                      targetNeuronName=4, weight=0.1)
        # pyrosim.Send_Synapse(sourceNeuronName=0,
        #                      targetNeuronName=4, weight=0.1)
        # pyrosim.Send_Synapse(sourceNeuronName=1,
        #                      targetNeuronName=4, weight=4.0)
        # pyrosim.Send_Synapse(sourceNeuronName=2,
        #                      targetNeuronName=3, weight=0.1)

        for currentRow in range(0, c.numSensorNeurons):
            for currentColumn in range(0, c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                     targetNeuronName=currentColumn+c.numSensorNeurons, weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, c.numSensorNeurons-1)
        randomColumn = random.randint(0, c.numMotorNeurons-1)

        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, ID):
        self.myID = ID
