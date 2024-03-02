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

  private final AutoSubsystem autoSubsystem;
  NavigationSubsystem navSub;
  DriveSubsystem driveSub;
  double radians;

  PIDController rotatePID = new PIDController(0.63, 0, 0);

  MoveToCommand command = new MoveToCommand(1, 1, null);

  public RotateRobotCommand(AutoSubsystem autoSubsystem, DriveSubsystem driveSub, NavigationSubsystem navSub, double radians) {
    addRequirements(navSub, driveSub);
    this.autoSubsystem = autoSubsystem;
    this.navSub = navSub;
    this.driveSub = driveSub;
    this.radians = radians;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotatePID.setSetpoint(radians);
    rotatePID.setTolerance(0.001);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double speedOfRotation = rotatePID.calculate(navSub.angle);   //navSub.angle is the angle of the robot
        speedOfRotation = MathUtil.clamp(speedOfRotation, -0.7, 0.7);
        // rotateWheel(driveSub.flMotor, Math.PI/4);
        // rotateWheel(driveSub.flMotor, 3 *Math.PI/4);
        // rotateWheel(driveSub.flMotor, -3 * Math.PI/4);
        // rotateWheel(driveSub.flMotor, -Math.PI/4);
        driveSub.flMotor.driveMotor.set(speedOfRotation);
        driveSub.frMotor.driveMotor.set(speedOfRotation);
        driveSub.blMotor.driveMotor.set(speedOfRotation);
        driveSub.brMotor.driveMotor.set(speedOfRotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rotatePID.atSetpoint();
  }
}
