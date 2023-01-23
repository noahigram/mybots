import pyrosim.pyrosim as pyrosim
import constants as c
import numpy as np
import pybullet as p


class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        # self.Prepare_To_Act()

    # def Prepare_To_Act(self):
    #     self.amplitude = c.amplitudeBack
    #     self.frequency = c.frequencyFront
    #     self.phaseOffset = c.phaseOffsetBack
    #     if self.jointName == "Torso_FrontLeg":
    #         self.targetAnglesFront = self.amplitude * \
    #             np.sin(self.frequency*np.linspace(0, 2 *
    #                                               np.pi, c.simLength)+self.phaseOffset)
    #     else:
    #         self.targetAnglesFront = self.amplitude * \
    #             np.sin((self.frequency/2)*np.linspace(0, 2 *
    #                                                   np.pi, c.simLength)+self.phaseOffset)

    def Set_Value(self, robot, desiredAngle):

        pyrosim.Set_Motor_For_Joint(
            bodyIndex=robot.robotID,
            jointName=self.jointName,
            controlMode=p.POSITION_CONTROL,
            targetPosition=desiredAngle,
            maxForce=c.backLegForce
        )
