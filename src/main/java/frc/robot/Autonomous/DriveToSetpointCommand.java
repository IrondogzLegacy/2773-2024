// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Information.OdometrySubsystem;

public class DriveToSetpointCommand extends Command {
  /** Creates a new DriveToSetpointCommand. */
  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;
  int i;
  double speed;
  public DriveToSetpointCommand(int i, double speed, DriveSubsystem driveSub, OdometrySubsystem odomSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.odomSub = odomSub;
    this.driveSub = driveSub;
    this.i = i;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.directionalDrive(0.3 * speed, -Math.PI/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (odomSub.setX[i] - odomSub.x) < 0.1 && (odomSub.setY[i] - odomSub.y) < 0.1;
  }
}
