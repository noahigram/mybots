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

        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.5)

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

       # Snake
       # SNake should have a random number of randomly shaped links, with random sensor placement along the chain
        # First make the head

       # Create a loop that iterates a random number of times, 1-20
        self.numLinks = random.randint(6, 12)

        # Decide which links will get sensor neurons
        self.sensorVec = np.random.randint(0, 2, self.numLinks)
        print(self.sensorVec)

        for link in range(0, self.numLinks):
            if self.sensorVec[link] == 0:
                linkCol = '<color rgba = "0 0 1 1"/>'
                colName = 'Blue'
            else:
                linkCol = '<color rgba = "0 1 0 1"/>'
                colName = 'Green'

            if link == 0:

                linkSize = [0.6, 0.6, 0.6]
                linkPos = [0, 0, 0.5]

                pyrosim.Send_Cube(name="0", pos=linkPos, size=linkSize,
                                  colorString=linkCol, colorName=colName)

            else:
                if link == 1:
                    jointPos = [linkSize[0]/2, 0, linkPos[2]/2]
                    pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
                        link), type="revolute", position=jointPos, jointAxis="0 0 1")
                else:
                    jointPos = [linkSize[0], 0, 0]
                    if random.randint(0, 1) == 0:
                        axis = "0 0 1"
                    else:
                        axis = "0 1 0"

                    pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
                        link), type="revolute", position=jointPos, jointAxis=axis)

                linkSize = [random.uniform(0.2, 0.5), random.uniform(
                    0.2, 0.5), random.uniform(0.2, 0.5)]
                linkPos = [linkSize[0]/2, 0, 0]
                pyrosim.Send_Cube(name=str(
                    link), pos=linkPos, size=linkSize, colorString=linkCol, colorName=colName)

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
        for link in range(0, self.numLinks-1):
            if self.sensorVec[link] == 1:
                pyrosim.Send_Sensor_Neuron(
                    name=link+self.numMotorNeurons, linkName=str(link))
                self.numSensorNeurons += 1

        print("Sensor Neurons: " + str(self.numSensorNeurons))
        print("Motor Neurons: " + str(self.numMotorNeurons))

        self.weights = np.random.rand(
            self.numSensorNeurons, self.numMotorNeurons)
        self.weights = self.weights * 2 - 1

        for currentRow in range(0, self.numSensorNeurons):
            for currentColumn in range(0, self.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=currentRow,
                                     targetNeuronName=currentColumn+self.numSensorNeurons, weight=self.weights[currentRow][currentColumn])
        pyrosim.End()

    def Mutate(self):
        if self.numMotorNeurons != 0 & self.numSensorNeurons != 0:
            randomRow = random.randint(0, self.numSensorNeurons-1)
            randomColumn = random.randint(0, self.numMotorNeurons-1)

            self.weights[randomRow, randomColumn] = random.random() * 2 - 1

    def Set_ID(self, ID):
        self.myID = ID
