// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveDistanceAngleCommand extends Command {

  public MoveDistanceAngleCommand(NavigationSubsystem navigationSubsystem, DriveSubsystem driveSubsystem) {
    addRequirements(navigationSubsystem, driveSubsystem);
    this.navigationSubsystem = navigationSubsystem;
  }

  NavigationSubsystem navigationSubsystem;

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

  public void rotateTo(double radians) {
    while (navigationSubsystem.angle < radians) {

    }
  }
}
