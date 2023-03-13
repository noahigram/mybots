# Ludobots Final Project

By Noah Igram

This repo contains the code used for the artificial life final project. The goal of this project was to implement a parallel hill climber which evolves a random body throughout the generations. The most important parts of the code for this project are summarized below. Starter code for this gitHub repo was provided by the Ludobots MOOC on reddit. This MOOC can be found in the r/ludobots subreddit of Reddit.com.

## Instructions for using this code

To simulate the evolution of a population of robots, run search.py. Doing this will run the parallel hill climber code once with the specified number of generations and population size defined in constants.py. For safest use, use a population size of at most 10 and a number of generations of at most 400.

Once search.py has finished, it will prompt the user to press enter to continue. This will trigger the best and worst simulations to run side by side in two separate windows. The reason for including this stopping point is so that if a large number of generations is used, one can walk away and not worry about missing the final simulation.

A plot will also appear which shows the fitness of the best parent at each generation.

## Body and Brain Generation

## Solution.py

The solution.py script contains code which creates the simulation world, a random robot body, and random brain which belongs to the robot generated. Key parts of this script are summarized below, and at the end of this readMe there is a diagram demonstrating the setup of some random bodies that could be generated.

### Create_Body()

The Generate_Body() method creates a robot body with a random number of randomly shaped links. It creates each link by randomly choosing its size and connects it to a random side of the link before it. Links with sensors are colored green and links without sensors are colored blue.

### Create_Brain()

The Generate_Brain() method creates a neural network with a corresponding nndf file. It first assigns a motor neuron to each joint, and then randomly assigns sensor neurons to some of the joints. Lastly, it connects each sensor neuron to each motor neuron with a random synaptic weight using pyrosim's Send_Synapse function.

## Body and Brain Evolution

### Body Evolution

Our goal for evolving the morphology of the robots across generations was to add a link to the robot at each generation. I had a lot of trouble implementing this process because I worked off of my parallel hill climber code from assignment 4, which only used a single urdf file. I was able to at least generate a new random robot at each step, but I ran into problems trying to make the robot evolve from its parent. Below is a diagram of the body generation.![Random3DBody](https://user-images.githubusercontent.com/75544386/220231503-1c5ac866-97dd-407c-8d4e-9a28791b0224.jpg)

### Brain Evolution

As each new child is created in the simulation, the synaptic weight of one of its neurons is randomly changed. This affects the fitness of the robot and will eventually decide whether it lives or dies.

<img width="973" alt="Screen Shot 2023-02-28 at 8 38 56 PM" src="https://user-images.githubusercontent.com/75544386/222031768-7ce43c31-dde6-49ab-91ad-49a3081c25ef.png">
