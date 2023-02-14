import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time
import constants as c


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID

    def Start_Simulation(self, runString):
        os.system("python3 simulate.py " + runString +
                  " " + str(self.myID) + " 2&>1 &")
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = f"fitness{self.myID}.txt"
        f = open(fitnessFileName, "r")

        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.5)

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

       # Snake
       # SNake should have a random number of randomly shaped links, with random sensor placement along the chain
        # First make the head

       # Create a loop that iterates a random number of times, 1-20
        self.numLinks = random.randint(2, 20)
        for link in range(0, self.numLinks):
            if link == 0:
                pos = [0, 0, 1]
                size = [random.uniform(0.2, 0.5), random.uniform(
                    0.2, 0.5), random.uniform(0.2, 0.5)]
                pyrosim.Send_Cube(name="0", pos=pos, size=size)

            else:
                pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
                    link), type="revolute", position=[pos[0]+size[0]/2, 0, 1], jointAxis="0 0 1")
                size = [random.uniform(0.2, 0.5), random.uniform(
                    0.2, 0.5), random.uniform(0.2, 0.5)]
                pos = [pos[0]+size[0]/2, 0, 1]
                pyrosim.Send_Cube(name=str(link), pos=pos, size=size)
            print(link)

        # Head
        # pyrosim.Send_Cube(name="Head", pos=[0, 0, 0], size=[0.3, 0.2, 0.2])
        # pyrosim.Send_Joint(name="Torso_Head", parent="Torso", child="Head",
        #                    type="revolute", position=[-0.5, 0, 1], jointAxis="1 0 0")

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
        # Loop through links , assign a motor neuron to each joint, and randomly assign sensor neurons to some of the joints
        self.numSensorNeurons = 0
        self.numMotorNeurons = 0
        for link in range(0, self.numLinks-1):
            pyrosim.Send_Motor_Neuron(
                name=str(link)+"_"+str(link+1), jointName=str(link)+"_"+str(link+1))
            self.numMotorNeurons += 1
            if random.randint(0, 1) == 0:
                pyrosim.Send_Sensor_Neuron(
                    name=link, linkName=str(link))
                self.numSensorNeurons += 1

        # pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
        # pyrosim.Send_Sensor_Neuron(name=1, linkName="FrontLeftLeg")
        # pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontRightLeg")
        # pyrosim.Send_Sensor_Neuron(name=3, linkName="BackLeftLeg")
        # pyrosim.Send_Sensor_Neuron(name=4, linkName="BackRightLeg")

        self.weights = np.random.rand(
            self.numSensorNeurons, self.numMotorNeurons)
        self.weights = self.weights * 2 - 1

        for currentRow in range(0, self.numSensorNeurons):
            for currentColumn in range(0, self.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                     targetNeuronName=currentColumn, weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        randomRow = random.randint(0, self.numSensorNeurons-1)
        randomColumn = random.randint(0, self.numMotorNeurons-1)

        self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, ID):
        self.myID = ID
