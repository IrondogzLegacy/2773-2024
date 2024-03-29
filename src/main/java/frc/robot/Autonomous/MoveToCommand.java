// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Navigation.OdometrySubsystem;

public class MoveToCommand extends Command {

  OdometrySubsystem odometrySubsystem;
  DriveSubsystem driveSubsystem;
  double x;
  double y;
  double goalX;
  double goalY;
  double differenceX;
  double differenceY;
  double distance = Math.sqrt(differenceX * differenceX + differenceY * differenceY);
  double radians = Math.atan(differenceY / differenceX);
  double initialX;
  double initialY;

  PIDController pid = new PIDController(0.63, 0, 0);
  double tolerance = 0.01;
  double speed;

  /** Creates a new MoveToCommand. */
  public MoveToCommand(double x, double y, OdometrySubsystem odometrySubsystem, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem, odometrySubsystem);
    this.driveSubsystem = driveSubsystem;
    this.odometrySubsystem = odometrySubsystem;
    this.goalX = x;
    this.goalY = y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(0);
    pid.setTolerance(tolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = odometrySubsystem.x;
    y = odometrySubsystem.y;
    differenceX = -(x - goalX);
    differenceY = -(y - goalY);
    radians = Math.atan2(differenceY, differenceX);
    distance = (differenceX * differenceX + differenceY * differenceY);

    speed = MathUtil.clamp(-pid.calculate(distance), -0.3, 0.3);
    driveSubsystem.directionalDrive(speed, radians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    System.out.println("Stopped Move To");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
