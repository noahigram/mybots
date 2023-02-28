import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import random
import time


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID
        # self.weights = np.random.rand(3, 2)
        # self.weights = self.weights * 2 - 1
        self.seed = np.random.randint(1, 10000)
        np.random.seed(self.seed)
        self.mutations = 0

        # Set synapse weights randomly- need numbers of sensor and motor neurons
        # Decide which links will get sensors:
        self.numLinks = np.random.randint(3, 12)
        # 0 if no sensor, 1 if sensor
        self.sensorVec = np.random.randint(0, 2, self.numLinks)
        self.numSensorNeurons = sum(self.sensorVec)
        self.numMotorNeurons = self.numLinks-1
        self.weights = np.random.rand(
            self.numSensorNeurons, self.numMotorNeurons)
        self.weights = self.weights * 2 - 1

    def Start_Simulation(self, runString):
        os.system("python3 simulate.py " + runString +
                  " " + str(self.myID) + " &")
        self.Create_World()
        self.Create_Body()
        self.Create_Brain()

    def Wait_For_Simulation_To_End(self):
        fitnessFileName = "fitness"+str(self.myID)+".txt"
        while not os.path.exists(fitnessFileName):
            time.sleep(0.01)

        f = open("fitness"+str(self.myID)+".txt", "r")
        self.fitness = float(f.read())

        f.close()
        os.system("rm fitness"+str(self.myID)+".txt")

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[4, 4, 0.5], size=[1, 1, 1])
        pyrosim.End()

    def Create_Body(self):

        pyrosim.Start_URDF("body"+str(self.myID)+".urdf")

        lastFace = 1
        random.seed(self.seed)

        # Create lists for storing link and joint positions (absolute) and link sizes
        absLinkPos = []
        absJointPos = []
        linkSizes = []

        # Loop through links, create links and connect them with joints
        for link in range(0, self.numLinks):
            # Assign blue or green to link
            if self.sensorVec[link] == 0:
                linkCol = '<color rgba = "0 0 1 1"/>'
                colName = 'Blue'
            else:
                linkCol = '<color rgba = "0 1 0 1"/>'
                colName = 'Green'

            # First link we create with a set size and it gets absolute positioning
            if link == 0:
                linkSizes.append([0.4, 0.4, 0.4])
                linkPos = [0, 0, 1.5]
                absLinkPos = linkPos
                pyrosim.Send_Cube(
                    name="0", pos=linkPos, size=linkSizes[link], colorString=linkCol, colorName=colName)

            # Rest of the links get relative positioning
            else:
                # But first joint gets absolute positioning
                if link == 1:
                    # Create first joint
                    absJointPos = [linkSizes[0][0]/2, 0, absLinkPos[2]]
                    pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(
                        link-1), child=str(link), type="revolute", position=absJointPos, jointAxis="0 0 1")
                    # Create next link of random size
                    linkSizes.append([random.uniform(0.2, 0.5), random.uniform(
                        0.2, 0.5), random.uniform(0.2, 0.5)])
                    linkPos = [linkSizes[1][0]/2, 0, 0]
                    pyrosim.Send_Cube(name=str(
                        link), pos=linkPos, size=linkSizes[1], colorString=linkCol, colorName=colName)

                    # Update current link's absolute position
                    absLinkPos = [linkPos[0]+absLinkPos[0],
                                  linkPos[1]+absLinkPos[1], linkPos[2]+absLinkPos[2]]

                # For the rest of the links we add the next joint to a random face of the link and create a new link in that direction
                else:
                    # Randomly choose a joint axis
                    randAxis = random.randint(0, 2)
                    if randAxis == 0:
                        axis = "0 0 1"
                    elif randAxis == 1:
                        axis = "1 0 0"
                    else:
                        axis = "0 1 0"

                    # Generate next link's size
                    lastLinkSize = linkSizes[link-1]
                    linkSizes.append([random.uniform(0.2, 0.3), random.uniform(
                        0.4, 0.5), random.uniform(0.4, 0.7)])
                    currentLinkSize = linkSizes[link]

                    # Choose joint position as a random face of the last link
                    faceNum = random.randint(1, 3)

                    flip = random.randint(1, 2)
                    if flip == 1:
                        faceNum = -faceNum
                    while faceNum == -lastFace:
                        faceNum = random.randint(1, 3)
                        # Don't go in the direction of the last link
                        flip = random.randint(1, 2)
                        if flip == 1:
                            faceNum = -faceNum

                    # Need distance from center of last link to last joint
                    dCenter = [absLinkPos[0] - absJointPos[0], absLinkPos[1] -
                               absJointPos[1], absLinkPos[2] - absJointPos[2]]

                    if faceNum == 1:  # Add joint and link in positive x direction

                        jointPos = [lastLinkSize[0]/2 +
                                    dCenter[0], dCenter[1], dCenter[2]]
                        linkPos = [currentLinkSize[0]/2, 0, 0]
                        absJointPos = [absJointPos[0]+jointPos[0],
                                       absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
                    if faceNum == -1:  # Add joint and link in negative x direction
                        jointPos = [-lastLinkSize[0]/2 +
                                    dCenter[0], dCenter[1], dCenter[2]]
                        linkPos = [-currentLinkSize[0]/2, 0, 0]
                        absJointPos = [absJointPos[0]+jointPos[0],
                                       absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
                    if faceNum == 2:  # Add joint and link in positive y direction
                        jointPos = [dCenter[0], lastLinkSize[1]/2 +
                                    dCenter[1], dCenter[2]]
                        linkPos = [0, currentLinkSize[1]/2, 0]
                        absJointPos = [absJointPos[0]+jointPos[0],
                                       absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
                    if faceNum == -2:  # Add joint and link in negative y direction
                        jointPos = [dCenter[0], -
                                    lastLinkSize[1]/2+dCenter[1], dCenter[1]]
                        linkPos = [0, -currentLinkSize[1]/2, 0]
                        absJointPos = [absJointPos[0]+jointPos[0],
                                       absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
                    if faceNum == 3:  # Add joint and link in positive z direction
                        jointPos = [dCenter[0], dCenter[1],
                                    lastLinkSize[2]/2+dCenter[2]]
                        linkPos = [0, 0, currentLinkSize[2]/2]
                        absJointPos = [absJointPos[0]+jointPos[0],
                                       absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
                    if faceNum == -3:  # Add joint and link in negative z direction
                        jointPos = [dCenter[0], dCenter[1], -
                                    lastLinkSize[2]/2+dCenter[2]]
                        linkPos = [0, 0, -currentLinkSize[2]/2]
                        absJointPos = [absJointPos[0]+jointPos[0],
                                       absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
                    lastFace = faceNum

                    pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
                        link), type="revolute", position=jointPos, jointAxis=axis)
                    pyrosim.Send_Cube(name=str(
                        link), pos=linkPos, size=currentLinkSize, colorString=linkCol, colorName=colName)

                    # Update current link's absolute position
                    absLinkPos = [linkPos[0]+absLinkPos[0],
                                  linkPos[1]+absLinkPos[1], linkPos[2]+absLinkPos[2]]

        pyrosim.End()

    def Create_Brain(self):
        print(self.myID)
        pyrosim.Start_NeuralNetwork("brain"+str(self.myID)+".nndf")
        neuronsAssigned = 0
        # Assign sensor neurons first
        for link in range(0, self.numLinks-1):
            if self.sensorVec[link] == 1:
                pyrosim.Send_Sensor_Neuron(
                    name=neuronsAssigned, linkName=str(link))
                neuronsAssigned += 1

        # Assign motor neurons
        for link in range(0, self.numLinks-1):
            pyrosim.Send_Motor_Neuron(
                name=neuronsAssigned, jointName=str(link)+"_"+str(link+1))
            neuronsAssigned += 1

        for sensorNeuron in range(0, self.numSensorNeurons):
            for motorNeuron in range(self.numSensorNeurons, self.numSensorNeurons+self.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName=sensorNeuron, targetNeuronName=motorNeuron,
                                     weight=self.weights[sensorNeuron][motorNeuron-self.numSensorNeurons])

        pyrosim.End()

    def Mutate(self, mutations):

        np.random.seed(mutations)

        # Randomly update a synapse weight
        randomRow = np.random.randint(0, self.numSensorNeurons-1)
        randomColumn = np.random.randint(0, self.numMotorNeurons-1)

        self.weights[randomRow, randomColumn] = np.random.random() * 2 - 1

        # Randomly change morphology of robot

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID
