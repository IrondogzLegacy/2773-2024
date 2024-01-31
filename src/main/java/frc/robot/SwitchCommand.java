// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class SwitchCommand extends Command {
  DriveSubsystem driveSubsystem;
  Command command1;
  Command command2;
  /** Creates a new SwitchCommand. */
  public SwitchCommand(DriveSubsystem driveSubsystem, Command command1, Command command2) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.command1 = command1;
    this.command2 = command2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (driveSubsystem.getDefaultCommand() == command1) {
        driveSubsystem.setDefaultCommand(command2);
    } else {
      driveSubsystem.setDefaultCommand(command1);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
