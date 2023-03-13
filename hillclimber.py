from solution import SOLUTION
import constants as c
import copy
import random


class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()
        self.mutations = 0

    def Evolve_For_One_Generation(self):
        pass
        # self.Spawn()
        # self.Mutate()
        # self.child.Evaluate("DIRECT")
        # self.Print()
        # self.Select()
        # self.mutations += 1

    def Print(self):
        print("\n")
        print(self.parent.fitness, self.child.fitness)

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate(self.mutations)
        # print(self.parent.weights)
        # print(self.child.weights)

    def Select(self):
        if self.parent.fitness < self.child.fitness:
            self.parent = self.child

    def Evolve(self):
        self.parent.Evaluate("DIRECT")

        # (c.numberOfGenerations):
        for currentGeneration in range(c.numberOfGenerations):
            if currentGeneration == 0:
                self.parent.Evaluate("DIRECT")
            self.Evolve_For_One_Generation()

    def Show_Best(self):
        self.parent.Evaluate("GUI")
