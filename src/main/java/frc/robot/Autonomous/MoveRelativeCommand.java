// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveRelativeCommand extends Command {

  double radians;
  double distance;
  AutoSubsystem autoSub;
  double distanceY;
  double distanceX;
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
  public MoveRelativeCommand(double x, double y, AutoSubsystem autoSubsystem) {
    addRequirements(autoSubsystem);
    this.goalX = x;
    this.goalY = y;
    this.autoSub = autoSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    yDif = autoSub.navSub.y;
    xDif = autoSub.navSub.x;
    distanceY = goalY;
    distanceX = goalX;
    pid.setSetpoint(0);
    pid.setTolerance(tolerance);
  }

  // Called every time the schetduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = autoSub.navSub.x - xDif;
    y = autoSub.navSub.y - yDif;
    distanceX = -(x - goalX);
    distanceY = -(y - goalY);

    radians = distanceY/distanceX;

    distance = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
    speed = MathUtil.clamp(-pid.calculate(distance), -0.7, 0.7);

    autoSub.driveSub.directionalDrive(speed, radians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (pid.atSetpoint() || ((goalX + tolerance < x && x < goalX + tolerance) && (goalY + tolerance < y && y < goalY + tolerance)));
  }
}
