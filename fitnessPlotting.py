import search
import matplotlib.pyplot as plt
import numpy as np
import constants as c

gens = np.ones(c.numberOfGenerations)
plt.plot(gens, search.phc1.bestParentFitnesses, label="Seed 1")
plt.plot(gens, search.phc2.bestParentFitnesses, label="Seed 2")
plt.plot(gens, search.phc3.bestParentFitnesses, label="Seed = 3")
plt.plot(gens, search.phc4.bestParentFitnesses, label="Seed = 4")
plt.plot(gens, search.phc5.bestParentFitnesses, label="Seed = 5")
plt.show()
