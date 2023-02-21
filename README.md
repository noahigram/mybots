# Ludobots Assignment 7
By Noah Igram

This repo contains the code used for assignment 7. The goal of this assignment was to generate random bodies which take up 3 dimensional space in the simulation. The most important parts of the code for this project are summarized below. 


## Generate.py
The generate.py script contains code which creates the simulation world, a random robot body, and random brain which belongs to the robot generated. Key parts of this script are summarized below, and at the end of this readMe there is a diagram demonstrating the setup of some random bodies that could be generated.

### Generate_Body()
The Generate_Body() function creates a robot body with a random number of randomly shaped links. It first creates a URDF file with pyrosim's Start_URDF function. The URDF file of the most recent run of the simulation can be found in this repo as "body.urdf". Once it has randomly generated a number of links, it creates the first link located at the origin of the simulation. It then randomly chooses a face of the first link (rectangular prism) and creates a joint on this face of the link. Then it creates the next link by randomly choosing its size and connecting it to the joint that was just created. This process is repeated for the number of links decided before. 

### Generate_Brain()
The Generate_Brain() function creates a neural network with a corresponding nndf file. It first assigns a motor neuron to each joint, and then randomly assigns sensor neurons to some of the joints. Lastly, it connects each sensor neuron to each motor neuron with a random synaptic weight using pyrosim's Send_Synapse function. ![Random3DBody](https://user-images.githubusercontent.com/75544386/220231503-1c5ac866-97dd-407c-8d4e-9a28791b0224.jpg)
