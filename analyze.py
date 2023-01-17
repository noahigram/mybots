import numpy as np
import matplotlib.pyplot as plt
backLegSensorValues = np.load("data/backLegSensorValues.npy")
frontLegSensorValues = np.load("data/frontLegSensorValues.npy")
targetAnglesValuesBack = np.load("data/targetAnglesValuesBack.npy")
targetAnglesValuesFront = np.load("data/targetAnglesValuesFront.npy")
plt.plot(targetAnglesValuesBack, label="Back Leg Motor Values")
plt.plot(targetAnglesValuesFront, label="Front Leg Motor Values")
# plt.plot(backLegSensorValues, label="Back Leg", linewidth=4)
# plt.plot(frontLegSensorValues, label="Front Leg")
plt.legend()
plt.show()
