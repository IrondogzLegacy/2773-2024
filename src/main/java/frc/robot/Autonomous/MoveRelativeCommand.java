// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Navigation.OdometrySubsystem;

public class MoveRelativeCommand extends Command {

  double radians;
  double distance;
  OdometrySubsystem odometrySubsystem;
  DriveSubsystem driveSubsystem;
  double differenceY;
  double differenceX;
  double goalX;
  double goalY;
  double x;
  double y;
  double xDif;
  double yDif;
  final double tolerance = 0.05;     //Tolerance of fianl position coordinate in meters
  double speed;
  PIDController pid = new PIDController(0.63, 0, 0);
  

  /** Creates a new MovePolarCommand. */
  public MoveRelativeCommand(double x, double y, OdometrySubsystem odometrySubsystem, DriveSubsystem driveSubsystem) {
    addRequirements(driveSubsystem, odometrySubsystem);
    this.goalX = x;
    this.goalY = y;
    this.driveSubsystem = driveSubsystem;
    this.odometrySubsystem = odometrySubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yDif = odometrySubsystem.y;
    xDif = odometrySubsystem.x;
    differenceY = goalY;
    differenceX = goalX;
    pid.setSetpoint(0);
    pid.setTolerance(tolerance);
  }

  // Called every time the schetduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = odometrySubsystem.x - xDif;
    y = odometrySubsystem.y - yDif;
    differenceX = -(x - goalX);
    differenceY = -(y - goalY);

    radians = Math.atan2(differenceY, differenceX);

    distance = Math.sqrt(differenceX * differenceX + differenceY * differenceY);
    speed = MathUtil.clamp(-pid.calculate(distance), -0.3, 0.3);

    driveSubsystem.directionalDrive(speed, radians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stop();
    System.out.println("Stopped Move Relative");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
