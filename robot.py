# TODO: insert robot code here
#!/usr/bin/env python3
"""
    This is a good foundation to build your robot code on
"""

import wpilib
import wpilib.drive

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.frontLeft = wpilib.Spark(20)
        self.rearLeft = wpilib.Spark(13)
        self.left = wpilib.SpeedControllerGroup(self.frontLeft, self.rearLeft)

        self.frontRight = wpilib.Spark(14)
        self.rearRight = wpilib.Spark(15)
        self.right = wpilib.SpeedControllerGroup(self.frontRight, self.rearRight)

        self.drive = wpilib.drive.DifferentialDrive(self.left, self.right)
        self.stick = wpilib.Joystick(1)
        self.timer = wpilib.Timer()

    # def autonomousInit(self):
    #     """This function is run once each time the robot enters autonomous mode."""
    #     self.timer.reset()
    #     self.timer.start()

    # def autonomousPeriodic(self):
    #     """This function is called periodically during autonomous."""

    #     # Drive for two seconds
    #     if self.timer.get() < 2.0:
    #         self.drive.arcadeDrive(-0.5, 0)  # Drive forwards at half speed
    #     else:
    #         self.drive.arcadeDrive(0, 0)  # Stop robot

    def teleopPeriodic(self):
        """This function is called periodically during operator control."""
        self.drive.arcadeDrive(self.stick.getY(), self.stick.getX())


if __name__ == "__main__":
    wpilib.run(MyRobot)