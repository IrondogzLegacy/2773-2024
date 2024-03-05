// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class MovePolarCommand extends Command {

  double radians;
  double distance;
  AutoSubsystem autoSub;
  double differenceY;
  double differenceX;
  double goalX;
  double goalY;
  double x;
  double y;
  final double tolerance = 0.05;     //Tolerance of fianl position coordinate in meters
  double speed;
  PIDController pid = new PIDController(0.63, 0, 0);
  

  /** Creates a new MovePolarCommand. */
  public MovePolarCommand(double radians, double distance, AutoSubsystem autoSubsystem) {
    addRequirements(autoSubsystem);
    this.radians = radians;
    this.distance = distance;
    this.autoSub = autoSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    differenceY = Math.sin(radians) * distance;
    differenceX = Math.cos(radians) * distance;
    y = autoSub.navSub.y;
    x = autoSub.navSub.x;
    goalY = y + differenceY;
    goalX = x + differenceX;
    pid.setSetpoint(0);
    pid.setTolerance(tolerance);
  }

  // Called every time the schetduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = autoSub.navSub.x;
    y = autoSub.navSub.y;
    differenceX = -(x - goalX);
    differenceY = -(y - goalY);

    radians = differenceY/differenceX;

    distance = Math.sqrt(differenceX * differenceX + differenceY * differenceY);
    speed = MathUtil.clamp(-pid.calculate(distance), -0.7, 0.7);

    autoSub.driveSub.directionalDrive(speed, radians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
