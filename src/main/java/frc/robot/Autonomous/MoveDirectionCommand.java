// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Information.OdometrySubsystem;

public class MoveDirectionCommand extends Command {

  OdometrySubsystem odometrySubsystem;
  DriveSubsystem driveSubsystem;
  double radians;

  /** Creates a new MoveDirectionCommand. */
  public MoveDirectionCommand(double radians, DriveSubsystem driveSubsystem, OdometrySubsystem odometrySubsystem) {
    addRequirements(driveSubsystem, odometrySubsystem);
    this.driveSubsystem = driveSubsystem;
    this.odometrySubsystem = odometrySubsystem;
    this.radians = radians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.directionalDrive(0.3, radians);
    System.out.println(odometrySubsystem.x + " , " + odometrySubsystem.y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    System.out.println("MoveDirection Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
