import pyrosim.pyrosim as pyrosim
import constants as c


class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.Prepare_To_Act()

    def Prepare_To_Act(self):
        self.amplitude = c.amplitudeBack
        self.frequency = c.frequencyBack
        self.phaseOffset = c.phaseOffsetBack
        # pyrosim.Set_Motor_For_Joint(
        #     bodyIndex=robotID,
        #     jointName=b'Torso_BackLeg',
        #     controlMode=p.POSITION_CONTROL,
        #     targetPosition=targetAnglesBack[ii],
        #     maxForce=c.backLegForce
        # )
        # pyrosim.Set_Motor_For_Joint(
        #     bodyIndex=robotID,
        #     jointName=b'Torso_FrontLeg',
        #     controlMode=p.POSITION_CONTROL,
        #     targetPosition=targetAnglesFront[ii],
        #     maxForce=c.frontLegForce
        # )
