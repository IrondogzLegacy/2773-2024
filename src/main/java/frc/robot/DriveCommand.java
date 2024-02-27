// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Navigation.*;

public class DriveCommand extends Command {
  DriveSubsystem driveSubsystem;
  XboxController joy;
  NavigationSubsystem navigationSubsystem;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, XboxController joy, NavigationSubsystem navigationSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.joy = joy;
    this.navigationSubsystem = navigationSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double Deadzone = Constants.ControllerDeadzone;
    double x = joy.getLeftX(), y = -joy.getLeftY();
    double speed = Math.sqrt(x * x + y * y) * Constants.DriveSpeedMultiplier;
    double angle = Math.atan2(y, x);
    double gyroAngle = navigationSubsystem.angle();
    // if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
    //   double rotate = joy.getRightX();
    //   if (Math.abs(rotate) > Deadzone) {
    //     driveSubsystem.rotate(Constants.RotateSpeedMultiplier * rotate);
    //   } else {
    //     driveSubsystem.stop();
    //   }
    // } else {
    //   driveSubsystem.directionalDrive(speed, angle - gyroAngle);
    // }

    double r = joy.getRightX();
    if (Math.abs(x) < Deadzone && Math.abs(y) < Deadzone && Math.abs(r) < Deadzone) {
      driveSubsystem.stop();
    } else {
      driveSubsystem.directionalDrive(speed, -angle + gyroAngle, Constants.RotateSpeedMultiplier * r);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
