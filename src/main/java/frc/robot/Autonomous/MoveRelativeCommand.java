// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;

public class MoveRelativeCommand extends Command {

  AutoSubsystem autoSubsystem;
  DriveSubsystem driveSubsystem;
  double x;
  double y;

  /** Creates a new MoveRelativeCommand. */
  public MoveRelativeCommand(double x, double y, AutoSubsystem autoSubsystem) {
    this.autoSubsystem = autoSubsystem;
    this.x = x;
    this.y = y;
    addRequirements(autoSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoSubsystem.moveRelative(x, y);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
