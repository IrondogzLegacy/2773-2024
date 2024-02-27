// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class CarDriveCommand extends Command {
  private DriveSubsystem driveSubsystem;
  private XboxController joy;

  /** Creates a new CarDriveCommand. */
  public CarDriveCommand(DriveSubsystem driveSubsystem, XboxController joy) {
    this.driveSubsystem = driveSubsystem;
    this.joy = joy;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.carDrive(joy.getRightX(), joy.getLeftY() * 0.5);
  }

  // Called once the com+mand ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
