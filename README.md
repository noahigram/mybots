# Ludobots Final Project

By Noah Igram

This repo contains the code used for the artificial life final project. The goal of this project was to implement a parallel hill climber which evolves a random body throughout the generations. The most important parts of the code for this project are summarized below. Starter code for this gitHub repo was provided by the Ludobots MOOC on reddit. This MOOC can be found in the r/ludobots subreddit of Reddit.com.

## Overview of this project
The main goal of this project was to evolve a population of robots to be able to move somewhat swiftly and quickly through their worlds. The parallel hill climber implements this in the following way:
1. An initial population of random robots is created - this is the first group of parents.
2. The fitness of each parent is measured as its distance travelled in the x direction.
3. A new child is spawned from each parent with a new brain and body
4. The fitness of each child is measured. If a child's fitness exceeds its parents fitness, it replaces the parent in the current generation.
5. This is repeated for the number of generations specified in constants.py.

By the end of these iterations, the final population of robots should have significantly better locomotion skills than the earlier generations.

We have included information on the code used for this project. There is also a video summarizing results at the end of this ReadMe file, as well as diagrams which outline how bodies and brains are generated and evolved. 

## Instructions for using this code

To simulate the evolution of a population of robots, run search.py. Doing this will run the parallel hill climber code once with the specified number of generations and population size defined in constants.py. For safest use, use a population size of at most 10 and a number of generations of at most 400. If search.py stops running, it is likely due to a memory error. This should not happen, but if it does, please rerun search.py with a new number of generations and population size by reducing them in constants.py.

Once search.py has finished, it will prompt the user to press enter to continue. This will trigger the best and worst simulations to run side by side in two separate windows. The reason for including this stopping point is so that if a large number of generations is used, one can walk away and not worry about missing the final simulation.

A plot will also appear which shows the fitness of the best parent at each generation.

## Body and Brain Generation

## Solution.py

The solution.py script contains code which creates the simulation world, a random robot body, and random brain which belongs to the robot generated. Key parts of this script are summarized below, and at the end of this readMe there is a diagram demonstrating the setup of some random bodies that could be generated.

### Create_Body()

The Generate_Body() method creates a robot body with a random number of randomly shaped links. It creates each link by randomly choosing its size and connects it to a random side of the link before it. Links with sensors are colored green and links without sensors are colored blue.

### Create_Brain()

The Generate_Brain() method creates a neural network with a corresponding nndf file. It first assigns a motor neuron to each joint, and then randomly assigns sensor neurons to some of the joints. Lastly, it connects each sensor neuron to each motor neuron with a random synaptic weight using pyrosim's Send_Synapse function.

## Parallel Hillclimber Implementation
The script parallelhillclimber.py contains the parallelhillclimber class which is instantiated at the beginning of search.py. Its purpose is to do the following at each generation:
1. Spawn a new population of children from the parent population. This is done in the Spawn() method. Diagrams showing the possible children brains and bodies can be found below in the body and brain evolution sections.
2. Mutate the children to randomly alter its brain. This is done in the Mutate() method. 
3. Run a simulation for each child, and record its fitness. This is done with the Evaluate() method. 
4. Compare the child's fitness to its parents. If it has a better fitness, it replaces its parent. This is done in the Select() method.

Below is a diagram summarizing this process.
![IMG_0338](https://user-images.githubusercontent.com/75544386/224860153-714a1e32-be2e-43a5-8f67-f736bba2c96e.jpg)

## Body and Brain Evolution

### Body Evolution

Our goal for evolving the morphology of the robots across generations was to add a link to the robot at each generation. I had a lot of trouble implementing this process because I worked off of my parallel hill climber code from assignment 4, which only used a single urdf file. I was able to at least generate a new random robot at each step, but I ran into problems trying to make the robot evolve from its parent. Below is a diagram of the body generation as well as a diagram showing how phenotype comes from genotype.![Random3DBody](https://user-images.githubusercontent.com/75544386/220231503-1c5ac866-97dd-407c-8d4e-9a28791b0224.jpg)
![IMG_0335](https://user-images.githubusercontent.com/75544386/224856574-33b71b55-6df9-4e09-950e-d1d2cca42e66.jpg)

### Brain Evolution

As each new child is created in the simulation, the synaptic weight of one of its neurons is randomly changed. This affects the fitness of the robot and will eventually decide whether it lives or dies.

<img width="973" alt="Screen Shot 2023-02-28 at 8 38 56 PM" src="https://user-images.githubusercontent.com/75544386/222031768-7ce43c31-dde6-49ab-91ad-49a3081c25ef.png">

Below are some sample brains that could be generated for a new member of the population.
![IMG_0334](https://user-images.githubusercontent.com/75544386/224839153-5b815a1e-2f38-485f-ab2c-000c5321303b.jpg)

## Summary of Results
Our main takeaway from this project is that random evolution is definitely a viable way for a population to evolve over time, if given enough time. We found that although a new randomly generated body always has the potential to travel well, the best way of ensuring that at least a subset of the population performs well is to evolve the population for a very large number of generations. The random variations in the brains and bodies at each iteration of the hillclimber are relatively small changes, so running for a large number of generations helps to ensure compounding of these small changes towards a better moving robot. 

We also found that the robots that evolved to move swiftly exhibited movement strategies that are somewhat oscillatory. These robots rely on the patterns of movement that lead them forward the most, and so often an evolved robot would repeat the same movement of its limbs to generate a repetitive move forward.

These robots could surely be improved. As we learned above, our best results come from runs with a large number of generations and a large population size. To arrive at better results we could run our simulation for longer or for a greater population, but we were held back by our limitations in computational power/computer memory. There are definitely ways we could speed up our code which could allow us to do this. Additionally, techniques for modifying the brains and bodies at each generation could also be improved. The changes between robots at each generation are very random. In pursuing any sort of hypothesis or goal for our robots, we can always use inspiration from what we expect, which could help us make less arbitrary changes at each generation. 

## Summary Video
The video file was too big to embed in this readme, so a link is attached. 

https://youtu.be/4_p1T98Kjws

