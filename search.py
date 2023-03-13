import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
import constants as c
import numpy as np
import matplotlib.pyplot as plt


# for i in range(5):
#     os.system("python3 generate.py")
#     os.system("python3 simulate.py")

# Cleanup before running

os.system("rm fitness*.txt")
os.system("rm body*.urdf")
os.system("rm brain*.nndf")

phc1 = PARALLEL_HILL_CLIMBER()
phc1.Evolve()

# We are going to run the simulation for a large number of generations so we need to pause it when the simulations end so I don't miss the GUI sim


def pause():
    programPause = input("Press the <ENTER> key to continue...")


print("Finished simulations...")
pause()

phc1.Show_Best()


# Plot best parent fitness at each generation
gens = range(0, c.numberOfGenerations)
print(phc1.bestParentFitnesses)

plt.title("Best Parent Fitness vs. Generation")
plt.xlabel("Generation")
plt.ylabel("Fitness")


# phc2 = PARALLEL_HILL_CLIMBER()
# phc2.Evolve()
# phc2.Show_Best()


# os.system("rm fitness*.txt")
# os.system("rm body*.urdf")
# os.system("rm brain*.nndf")

# phc3 = PARALLEL_HILL_CLIMBER()
# phc3.Evolve()
# phc3.Show_Best()


# os.system("rm fitness*.txt")
# os.system("rm body*.urdf")
# os.system("rm brain*.nndf")

# phc4 = PARALLEL_HILL_CLIMBER()
# phc4.Evolve()
# phc4.Show_Best()


# os.system("rm fitness*.txt")
# os.system("rm body*.urdf")
# os.system("rm brain*.nndf")

# phc5 = PARALLEL_HILL_CLIMBER()
# phc5.Evolve()
# # phc5.Show_Best()
plt.figure()
plt.plot(gens, phc1.bestParentFitnesses, label="Seed 1")

plt.show()
os.system("rm fitness*.txt")
os.system("rm body*.urdf")
os.system("rm brain*.nndf")
