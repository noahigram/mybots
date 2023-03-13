import numpy as np

simLength = 1000
sleepInt = 1/60

# Simulation Gravity
gravX = 0
gravY = 0
gravZ = -100

# Front and back leg motor forces
backLegForce = 500
frontLegForce = 30


# Amplitude, frequency, phase offset
amplitudeFront = np.pi
frequencyFront = 100
phaseOffsetFront = np.pi/3
amplitudeBack = np.pi/4
frequencyBack = 40
phaseOffsetBack = 0

numberOfGenerations = 500
populationSize = 10
