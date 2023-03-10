from solution import SOLUTION
import constants as c
import copy
import random
import os
import numpy as np


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("rm brain*.nndf")
        os.system("rm fitness*.nndf")
        os.system("rm body*.urdf")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(0, c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

        self.bestParentFitnesses = np.zeros(c.numberOfGenerations)
        self.currentGen = 0
        self.mutations = 0

        # self.parent = SOLUTION()
        # self.mutations = 0

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Get_Best_Fitness()
        self.currentGen += 1

        # self.Print()
        self.Select()
        self.mutations += 1

    def Get_Best_Fitness(self):
        bestParentFitness = self.parents[0].fitness
        for parent in self.parents.keys():

            if self.parents[parent].fitness > bestParentFitness:
                bestParentFitness = self.parents[parent].fitness

        self.bestParentFitnesses[self.currentGen] = bestParentFitness

    def Print(self):
        print("\n")
        for key in self.parents.keys():
            print(self.parents[key].fitness, self.children[key].fitness)

        print("\n")

    def Spawn(self):
        self.children = {}
        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for key in self.children.keys():
            self.children[key].Mutate(self.mutations)

        # print(self.parent.weights)
        # print(self.child.weights)

    def Select(self):

        for key in self.children.keys():
            if self.parents[key].fitness < self.children[key].fitness:
                self.parents[key] = self.children[key]

    def Evolve(self):

        # self.parent.Evaluate("DIRECT")
        self.Evaluate(self.parents)

        # # (c.numberOfGenerations):
        for currentGeneration in range(c.numberOfGenerations):
            # if currentGeneration == 0:
            #     self.parents[parent].Start_Simulation("DIRECT")
            #     self.parents[parent].Wait_For_Simulation_To_End()
            self.Evolve_For_One_Generation()

    def Evaluate(self, solutions):
        for parent in solutions.keys():
            solutions[parent].Start_Simulation("DIRECT")
        for parent in solutions.keys():
            solutions[parent].Wait_For_Simulation_To_End()

    def Show_Best(self):

        bestParent = self.parents[0]
        worstParent = bestParent
        for parent in self.parents.keys():
            if self.parents[parent].fitness > bestParent.fitness:
                bestParent = self.parents[parent]
            if self.parents[parent].fitness < worstParent.fitness:
                worstParent = self.parents[parent]
        # Show best and worst parent
        worstParent.Start_Simulation("GUI")
        bestParent.Start_Simulation("GUI")

        # self.parent.Evaluate("GUI")
