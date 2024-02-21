// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoSubsystem;
import frc.robot.DriveSubsystem;

public class MoveDistanceAngleCommand extends Command {
  DriveSubsystem driveSubsystem = null;  // new DriveSubsystem();


  public MoveDistanceAngleCommand(AutoSubsystem ams) {
    this.ams = ams;
  }

  AutoSubsystem ams;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ams.movePolar(1, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
