// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveDistanceAngleCommand extends Command {

  public MoveDistanceAngleCommand(NavigationSubsystem navigationSubsystem, DriveSubsystem driveSubsystem) {
    addRequirements(navigationSubsystem, driveSubsystem);
    this.navigationSubsystem = navigationSubsystem;
    this.driveSubsystem = driveSubsystem;
  }

  NavigationSubsystem navigationSubsystem;
  DriveSubsystem driveSubsystem;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void rotateBotTo(double radians) {
    if ((navigationSubsystem.angle + Math.PI) < radians) {
      while (navigationSubsystem.angle > radians) {
        driveSubsystem.rotate(-0.1);
      }
    } else {
      while (navigationSubsystem.angle < radians) {
        driveSubsystem.rotate(0.1);
      }
    }
  }

  public void rotateWheelsTo(double radians) {
    if ((navigationSubsystem.fla + Math.PI) < radians) {
      while (navigationSubsystem.fla > radians) {
        driveSubsystem.rotate(-Constants.WHEEL_ROTATE_SPEED);
      }
    } else {
      while (navigationSubsystem.fla < radians) {
        driveSubsystem.rotate(Constants.WHEEL_ROTATE_SPEED);
      }
    }
  }

  public void movePolar(double radians, double distance) {
    rotateWheelsTo(radians);

    double y = Math.sin(radians) * distance;
    double x = Math.cos(radians) * distance;
    double cx = navigationSubsystem.flx - Math.cos(radians) * distance;
    double cy = navigationSubsystem.fly - Math.sin(radians) * distance;
    while (cy < y || cx < x) {
      driveSubsystem.directionalDrive(1, radians);
    }
  }
}
