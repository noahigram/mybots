import pyrosim.pyrosim as pyrosim
import random as rand
import numpy as np
import constants as c


def Create_World():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[4, 4, 0.5], size=[1, 1, 1])
    pyrosim.End()


def Generate_Body():
    pyrosim.Start_URDF("body.urdf")

    # Snake
    # SNake should have a random number of randomly shaped links, with random sensor placement along the chain
    # First make the head

    # Create a loop that iterates a random number of times, 1-20
    numLinks = c.numLinks

    # Decide which links will get sensor neurons
    sensorVec = np.random.randint(0, 2, numLinks)
    print(sensorVec)

    for link in range(0, numLinks):
        if sensorVec[link] == 0:
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
                if rand.randint(0, 1) == 0:
                    axis = "0 0 1"
                else:
                    axis = "0 1 0"

                pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
                    link), type="revolute", position=jointPos, jointAxis=axis)

            linkSize = [rand.uniform(0.2, 0.5), rand.uniform(
                0.2, 0.5), rand.uniform(0.2, 0.5)]
            linkPos = [linkSize[0]/2, 0, 0]
            pyrosim.Send_Cube(name=str(
                link), pos=linkPos, size=linkSize, colorString=linkCol, colorName=colName)

        # Head
        # pyrosim.Send_Cube(name="Head", pos=[0, 0, 0], size=[0.3, 0.2, 0.2])
        # pyrosim.Send_Joint(name="Torso_Head", parent="Torso", child="Head",
        #                    type="revolute", position=[-0.5, 0, 1], jointAxis="1 0 0")

    pyrosim.End()


# def Generate_Brain():
#     pyrosim.Start_NeuralNetwork("brain.nndf")
#     pyrosim.Send_Sensor_Neuron(name=0, linkName="Torso")
#     pyrosim.Send_Sensor_Neuron(name=1, linkName="BackLeg")
#     pyrosim.Send_Sensor_Neuron(name=2, linkName="FrontLeg")
#     pyrosim.Send_Motor_Neuron(name=3, jointName="Torso_BackLeg")
#     pyrosim.Send_Motor_Neuron(name=4, jointName="Torso_FrontLeg")
#     pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=3, weight=0.1)
#     pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=3, weight=0.1)
#     pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=4, weight=0.1)
#     pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=4, weight=0.1)
#     pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=4, weight=4.0)
#     pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=3, weight=0.1)

#     for i in range(0, 3):
#         for j in range(3, 5):
#             pyrosim.Send_Synapse(sourceNeuronName=i,
#                                  targetNeuronName=j, weight=rand.uniform(-1, 1))
#     pyrosim.End()

def Generate_Brain(self):
    pyrosim.Start_NeuralNetwork("brain.nndf")
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
                                 targetNeuronName=currentColumn+self.numSensorNeurons, weight=rand.uniform(-1, 1))
    pyrosim.End()


Generate_Body()
Generate_Brain()
Create_World()
# Create_Robot()

# def Create_Body(self):
#         pyrosim.Start_URDF("body.urdf")

#        # Snake
#        # SNake should have a random number of randomly shaped links, with random sensor placement along the chain
#         # First make the head

#        # Create a loop that iterates a random number of times, 1-20
#         self.numLinks = random.randint(6, 12)

#         # Decide which links will get sensor neurons
#         self.sensorVec = np.random.randint(0, 2, self.numLinks)
#         print(self.sensorVec)

#         for link in range(0, self.numLinks):
#             if self.sensorVec[link] == 0:
#                 linkCol = '<color rgba = "0 0 1 1"/>'
#                 colName = 'Blue'
#             else:
#                 linkCol = '<color rgba = "0 1 0 1"/>'
#                 colName = 'Green'

#             if link == 0:

#                 linkSize = [0.6, 0.6, 0.6]
#                 linkPos = [0, 0, 0.5]

#                 pyrosim.Send_Cube(name="0", pos=linkPos, size=linkSize,
#                                   colorString=linkCol, colorName=colName)

#             else:
#                 if link == 1:
#                     jointPos = [linkSize[0]/2, 0, linkPos[2]/2]
#                     pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
#                         link), type="revolute", position=jointPos, jointAxis="0 0 1")
#                 else:
#                     jointPos = [linkSize[0], 0, 0]
#                     if random.randint(0, 1) == 0:
#                         axis = "0 0 1"
#                     else:
#                         axis = "0 1 0"

#                     pyrosim.Send_Joint(name=str(link-1) + "_" + str(link), parent=str(link-1), child=str(
#                         link), type="revolute", position=jointPos, jointAxis=axis)

#                 linkSize = [random.uniform(0.2, 0.5), random.uniform(
#                     0.2, 0.5), random.uniform(0.2, 0.5)]
#                 linkPos = [linkSize[0]/2, 0, 0]
#                 pyrosim.Send_Cube(name=str(
#                     link), pos=linkPos, size=linkSize, colorString=linkCol, colorName=colName)

#         # Head
#         # pyrosim.Send_Cube(name="Head", pos=[0, 0, 0], size=[0.3, 0.2, 0.2])
#         # pyrosim.Send_Joint(name="Torso_Head", parent="Torso", child="Head",
#         #                    type="revolute", position=[-0.5, 0, 1], jointAxis="1 0 0")

#         pyrosim.End()

#     def Create_Brain(self):
#         pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")
#         # Loop through links , assign a motor neuron to each joint, and randomly assign sensor neurons to some of the joints
#         self.numSensorNeurons = 0
#         self.numMotorNeurons = 0
#         for link in range(0, self.numLinks-1):
#             pyrosim.Send_Motor_Neuron(
#                 name=str(link)+"_"+str(link+1), jointName=str(link)+"_"+str(link+1))
#             self.numMotorNeurons += 1
#         for link in range(0, self.numLinks-1):
#             if self.sensorVec[link] == 1:
#                 pyrosim.Send_Sensor_Neuron(
#                     name=link+self.numMotorNeurons, linkName=str(link))
#                 self.numSensorNeurons += 1

#         print("Sensor Neurons: " + str(self.numSensorNeurons))
#         print("Motor Neurons: " + str(self.numMotorNeurons))

#         self.weights = np.random.rand(
#             self.numSensorNeurons, self.numMotorNeurons)
#         self.weights = self.weights * 2 - 1

#         for currentRow in range(0, self.numSensorNeurons):
#             for currentColumn in range(0, self.numMotorNeurons):
#                 pyrosim.Send_Synapse(sourceNeuronName=currentRow,
#                                      targetNeuronName=currentColumn+self.numSensorNeurons, weight=self.weights[currentRow][currentColumn])
#         pyrosim.End()
