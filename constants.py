import numpy as np

simLength = 400
sleepInt = 1/60

# Simulation Gravity
gravX = 0
gravY = 0
gravZ = -9.8

# Front and back leg motor forces
backLegForce = 200
frontLegForce = 30


# Amplitude, frequency, phase offset
amplitudeFront = np.pi/3
frequencyFront = 100
phaseOffsetFront = np.pi/3
amplitudeBack = np.pi/4
frequencyBack = 40
phaseOffsetBack = 0

numberOfGenerations = 20
populationSize = 20
