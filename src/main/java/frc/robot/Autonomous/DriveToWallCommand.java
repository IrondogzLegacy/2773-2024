// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Information.OdometrySubsystem;

public class DriveToWallCommand extends Command {
  /** Creates a new DriveToWallCommand. */
  public DriveToWallCommand(double radians, DriveSubsystem driveSub, OdometrySubsystem odomSub) {
    radians = this.radians;
    driveSub = this.driveSub;
    odomSub = this.odomSub;
  }

  double radians;
  double last = 0;
  double max = 0;
  DriveSubsystem driveSub;
  OdometrySubsystem odomSub;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (odomSub.displacementX + odomSub.displacementY > last) {
      max = odomSub.displacementX + odomSub.displacementY;
    }
    last = odomSub.displacementX + odomSub.displacementY;
    driveSub.directionalDrive(0.3, radians);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (odomSub.displacementX + odomSub.displacementY < last - 0.05 || (odomSub.displacementX + odomSub.displacementY < 0.01 && odomSub.displacementX + odomSub.displacementY < max));
  }
}
