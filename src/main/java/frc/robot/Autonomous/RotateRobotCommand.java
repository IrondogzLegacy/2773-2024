// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Navigation.NavigationSubsystem;

public class RotateRobotCommand extends Command {

  NavigationSubsystem navSub;
  DriveSubsystem driveSub;
  double radians;

  PIDController rotatePID = new PIDController(0.63, 0, 0);

  public RotateRobotCommand(double radians, NavigationSubsystem navigationSubsystem, DriveSubsystem driveSubsystem) {
    addRequirements(navSub, driveSub);
    this.navSub = navigationSubsystem;
    this.driveSub = driveSubsystem;
    this.radians = radians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    radians += navSub.angle;
    if (radians > Math.PI) {
      radians -= Math.PI*2;
    } else if (radians < -Math.PI) {
      radians += Math.PI * 2;
    }
    rotatePID.setSetpoint(radians);
    rotatePID.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speedOfRotation = rotatePID.calculate(navSub.angle);
    speedOfRotation = MathUtil.clamp(speedOfRotation, -0.3, 0.3);
    driveSub.rotate(speedOfRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.stop();
    System.out.println("Rotate Robot Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePID.atSetpoint();
  }
}
