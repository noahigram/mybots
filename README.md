# Ludobots Assignment 7
By Noah Igram

This repo contains the code used for assignment 7. The goal of this assignment was to generate random bodies which take up 3 dimensional space in the simulation. The most important parts of the code for this project are summarized below. 


## Generate.py
The generate.py script contains code which creates the simulation world, a random robot body, and random brain which belongs to the robot generated. Key parts of this script are summarized below, and at the end of this readMe there is a diagram demonstrating the setup of some random bodies that could be generated.

### Generate_Body()
The Generate_Body() function creates a robot body with a random number of randomly shaped links. It creates each link by randomly choosing its size and connects it to a random side of the link before it. Links with sensors are colored green and links without sensors are colored blue.

### Generate_Brain()
The Generate_Brain() function creates a neural network with a corresponding nndf file. It first assigns a motor neuron to each joint, and then randomly assigns sensor neurons to some of the joints. Lastly, it connects each sensor neuron to each motor neuron with a random synaptic weight using pyrosim's Send_Synapse function. ![Random3DBody](https://user-images.githubusercontent.com/75544386/220231503-1c5ac866-97dd-407c-8d4e-9a28791b0224.jpg)
