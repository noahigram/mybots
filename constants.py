import numpy as np

simLength = 1000
sleepInt = 1/240

# Simulation Gravity
gravX = 0
gravY = 0
gravZ = -9.8

# Front and back leg motor forces
backLegForce = 30
frontLegForce = 30


# Amplitude, frequency, phase offset
amplitudeFront = np.pi/3
frequencyFront = 40
phaseOffsetFront = np.pi/3
amplitudeBack = np.pi/4
frequencyBack = 40
phaseOffsetBack = 0
