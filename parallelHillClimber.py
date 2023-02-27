from solution import SOLUTION
import constants as c
import copy
import random
import os


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        # os.system("rm brain*.nndf")
        os.system("rm fitness*.txt")
        self.parents = {}
        self.nextAvailableID = 0
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

        # self.parent = SOLUTION()

    def Evolve_For_One_Generation(self):
        self.Spawn()

        self.Mutate()
        # ************************** Evaluate is callued here check for bug **********************
        self.Evaluate(self.children)

        # self.Print()
        self.Select()

    def Evaluate(self, solutions):
        for key in solutions.keys():
            solutions[key].Start_Simulation("DIRECT")
            # For some reason fitness*.txt is either removed or is never created before this next line
        for key in solutions.keys():
            solutions[key].Wait_For_Simulation_To_End()
        # for key in solutions.keys():
        #     solutions[key].Wait_For_Simulation_To_End()

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

        # self.child = copy.deepcopy(self.parent)
        # self.child.Set_ID(self.nextAvailableID)
        # self.nextAvailableID += 1

    def Mutate(self):
        for key in self.children.keys():
            self.children[key].Mutate()
        # print(self.parent.weights)
        # print(self.child.weights)

    def Select(self):
        for key in self.children.keys():
            if self.parents[key].fitness > self.children[key].fitness:
                self.parents[key] = self.children[key]

    def Evolve(self):
        # ************************** Evaluate is callued here check for bug **********************
        self.Evaluate(self.parents)

        # self.parent.Evaluate("DIRECT")

        # # (c.numberOfGenerations):
        for currentGeneration in range(c.numberOfGenerations):
            #     if currentGeneration == 0:
            #         self.parent.Evaluate("GUI")
            self.Evolve_For_One_Generation()

    def Show_Best(self):
        bestParent = self.parents[0]
        for key in self.parents.keys():
            if self.parents[key].fitness < bestParent.fitness:
                bestParent = self.parents[key]

        bestParent.Start_Simulation("GUI")
