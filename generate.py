import pyrosim.pyrosim as pyrosim
import random as rand
import constants as c
import numpy as np


def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[4, 4, 0.5], size=[1, 1, 1])
    pyrosim.End()


def Generate_Body():
    pyrosim.Start_URDF("body.urdf")

    # Robot with 3 links and two joints (Torso, Backleg and frontleg)
    # pyrosim.Send_Cube(name="Torso", pos=[1.5, 0, 1.5], size=[1, 1, 1])

    # pyrosim.Send_Joint(name="Torso_BackLeg", parent="Torso",
    #                    child="BackLeg", type="revolute", position=[1, 0, 1])
    # pyrosim.Send_Cube(name="BackLeg", pos=[-0.5, 0, -0.5], size=[1, 1, 1])
    # pyrosim.Send_Joint(name="Torso_FrontLeg", parent="Torso",
    #                    child="FrontLeg", type="revolute", position=[2, 0, 1])
    # pyrosim.Send_Cube(name="FrontLeg", pos=[0.5, 0, -0.5], size=[1, 1, 1])

    # Random robot
    # Random number of links
    numLinks = c.numLinks

    # Decide which links will get sensor neurons

    lastFace = 1

    # Create lists for storing link and joint positions (absolute) and link sizes
    absLinkPos = []
    absJointPos = []
    linkSizes = []

    # Loop through links, create links and connect them with joints
    for link in range(0, numLinks):
        # Assign blue or green to link
        if c.sensorVec[link] == 0:
            linkCol = '<color rgba = "0 0 1 1"/>'
            colName = 'Blue'
        else:
            linkCol = '<color rgba = "0 1 0 1"/>'
            colName = 'Green'

        # First link we create with a set size and it gets absolute positioning
        if link == 0:
            linkSizes.append([0.4, 0.4, 0.4])
            linkPos = [0, 0, 0.5]
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
                linkSizes.append([rand.uniform(0.2, 0.5), rand.uniform(
                    0.2, 0.5), rand.uniform(0.2, 0.5)])
                linkPos = [linkSizes[1][0]/2, 0, 0]
                pyrosim.Send_Cube(name=str(
                    link), pos=linkPos, size=linkSizes[1], colorString=linkCol, colorName=colName)

                # Update current link's absolute position
                absLinkPos = [linkPos[0]+absLinkPos[0],
                              linkPos[1]+absLinkPos[1], linkPos[2]+absLinkPos[2]]

            # For the rest of the links we add the next joint to a random face of the link and create a new link in that direction
            else:
                # Randomly choose a joint axis
                randAxis = rand.randint(0, 2)
                if randAxis == 0:
                    axis = "0 0 1"
                elif randAxis == 1:
                    axis = "1 0 0"
                else:
                    axis = "0 1 0"

                # Generate next link's size
                lastLinkSize = linkSizes[link-1]
                linkSizes.append([rand.uniform(0.2, 0.4), rand.uniform(
                    0.2, 0.4), rand.uniform(0.2, 0.4)])
                currentLinkSize = linkSizes[link]

                # Choose joint position as a random face of the last link
                faceNum = rand.randint(1, 3)
                flip = rand.randint(1, 2)
                if flip == 1:
                    faceNum = -faceNum
                while faceNum != -lastFace:
                    faceNum = rand.randint(1, 3)
                    # Don't go in the direction of the last link
                    flip = rand.randint(1, 2)
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

    pyrosim.End()

# def Generate_Body():


#         else:
#             if link == 1:
#                 jointPos = [linkSizes[0][0]/2, 0, linkPos[2]/2]
#                 absJointPos = jointPos
#                 pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
#                     link), type="revolute", position=jointPos, jointAxis="0 0 1")
#                 linkSizes.append([rand.uniform(0.2, 0.5), rand.uniform(
#                     0.2, 0.5), rand.uniform(0.2, 0.5)])
#                 linkPos = [linkSizes[0][0]/2, 0, 0]
#                 pyrosim.Send_Cube(name=str(
#                     link), pos=linkPos, size=linkSizes[1], colorString=linkCol, colorName=colName)
#                 # Update current absolute position of link
#                 absLinkPos = [linkPos[0]+absLinkPos[0],
#                               linkPos[1]+absLinkPos[1], linkPos[2]+linkPos[2]]
#             else:


#                 # Choose joint position as a random face of the last link

#                 faceNum = rand.randint(0, 5)
#                 while faceNum == lastFace:
#                     faceNum = rand.randint(0, 5)

#                 # Need distance from center of link to last joint
#                 dCenter = [absLinkPos[0] - absJointPos[0], absLinkPos[1] -
#                            absJointPos[1], absLinkPos[2] - absJointPos[2]]

#                 if faceNum == 0 and lastFace != 1:  # Add joint and link in positive x direction

#                     jointPos = [lastLinkSize[0]/2 +
#                                 dCenter[0], dCenter[1], dCenter[2]]
#                     linkPos = [currentLinkSize[0]/2, 0, 0]
#                     absJointPos = [absJointPos[0]+jointPos[0],
#                                    absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
#                 if faceNum == 1 and lastFace != 0:  # Add joint and link in negative x direction
#                     jointPos = [-lastLinkSize[0]/2 +
#                                 dCenter[0], dCenter[1], dCenter[2]]
#                     linkPos = [-currentLinkSize[0]/2, 0, 0]
#                     absJointPos = [absJointPos[0]+jointPos[0],
#                                    absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
#                 if faceNum == 2 and lastFace != 3:  # Add joint and link in positive y direction
#                     jointPos = [dCenter[0], lastLinkSize[1]/2 +
#                                 dCenter[1], dCenter[2]]
#                     linkPos = [0, currentLinkSize[1]/2, 0]
#                     absJointPos = [absJointPos[0]+jointPos[0],
#                                    absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
#                 if faceNum == 3 and lastFace != 2:  # Add joint and link in negative y direction
#                     jointPos = [dCenter[0], -
#                                 lastLinkSize[1]/2+dCenter[1], dCenter[1]]
#                     linkPos = [0, -currentLinkSize[1]/2, 0]
#                     absJointPos = [absJointPos[0]+jointPos[0],
#                                    absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
#                 if faceNum == 4 and lastFace != 5:  # Add joint and link in positive z direction
#                     jointPos = [dCenter[0], dCenter[1],
#                                 lastLinkSize[2]/2+dCenter[2]]
#                     linkPos = [0, 0, currentLinkSize[2]/2]
#                     absJointPos = [absJointPos[0]+jointPos[0],
#                                    absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
#                 if faceNum == 5 and lastFace != 4:  # Add joint and link in negative z direction
#                     jointPos = [dCenter[0], dCenter[1], -
#                                 lastLinkSize[2]/2+dCenter[2]]
#                     linkPos = [0, 0, -currentLinkSize[2]/2]
#                     absJointPos = [absJointPos[0]+jointPos[0],
#                                    absJointPos[1]+jointPos[1], absJointPos[2]+jointPos[2]]
#                 lastFace = faceNum

#                 pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
#                     link), type="revolute", position=jointPos, jointAxis=axis)
#                 pyrosim.Send_Cube(name=str(
#                     link), pos=linkPos, size=currentLinkSize, colorString=linkCol, colorName=colName)

#     pyrosim.End()


def Generate_Brain():
    pyrosim.Start_NeuralNetwork("brain.nndf")
    # pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
    # pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
    # pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
    # pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
    # pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")
    # pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=3, weight=0.1)
    # pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=3, weight=0.1)
    # pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=4, weight=0.1)
    # pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=4, weight=0.1)
    # pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=4, weight=4.0)
    # pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=3, weight=0.1)

    # for i in range(0, 3):
    #     for j in range(3, 5):
    #         pyrosim.Send_Synapse(sourceNeuronName=i,
    #                              targetNeuronName=j, weight=rand.uniform(-1, 1))

    # Loop through links, assign a motor neuron to each joint, and randomly assign sensor neurons to some of the joints
    numSensorNeurons = 0
    numMotorNeurons = 0
    for link in range(0, c.numLinks-1):
        if c.sensorVec[link] == 1:
            pyrosim.Send_Sensor_Neuron(
                name=numSensorNeurons, linkName=str(link))
            numSensorNeurons += 1
    for link in range(0, c.numLinks-1):
        pyrosim.Send_Motor_Neuron(
            name=numSensorNeurons+numMotorNeurons, jointName=str(link)+"_"+str(link+1))
        numMotorNeurons += 1

    # Generate synapses for pairs of neurons
    for i in range(0, numSensorNeurons):
        for j in range(numSensorNeurons, numSensorNeurons+numMotorNeurons):
            pyrosim.Send_Synapse(
                sourceNeuronName=i, targetNeuronName=j, weight=rand.uniform(-1, 1))
    pyrosim.End()


Generate_Body()
Generate_Brain()
Create_World()
# Create_Robot()

# Code for random 3D body


# def Generate_Brain():
#     pyrosim.Start_NeuralNetwork("brain.nndf")
#     # Loop through links , assign a motor neuron to each joint, and randomly assign sensor neurons to some of the joints
#     numSensorNeurons = 0
#     numMotorNeurons = 0
#     for link in range(0, c.numLinks-1):
#         pyrosim.Send_Motor_Neuron(
#             name=str(link)+"_"+str(link+1), jointName=str(link)+"_"+str(link+1))
#         numMotorNeurons += 1
#     for link in range(0, c.numLinks-1):
#         if c.sensorVec[link] == 1:
#             pyrosim.Send_Sensor_Neuron(
#                 name=link+numMotorNeurons, linkName=str(link))
#             numSensorNeurons += 1

#     for currentRow in range(0, numSensorNeurons):
#         for currentColumn in range(0, numMotorNeurons):
#             pyrosim.Send_Synapse(sourceNeuronName=currentRow,
#                                  targetNeuronName=currentColumn+numSensorNeurons, weight=rand.uniform(-1, 1))
#     pyrosim.End()
