// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveToCommand extends Command {

  AutoSubsystem ams;
  double x;
  double y;

  /** Creates a new MoveToCommand. */
  public MoveToCommand(double x, double y, AutoSubsystem autoSubsystem) {
    addRequirements(autoSubsystem);
    this.ams = autoSubsystem;
    this.x = x;
    this.y = y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ams.moveTo(x, y);
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
    return false;
  }
}