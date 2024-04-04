// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Autonomous.RotateRobotCommand;
import frc.robot.Information.TagSubsystem.TagData;

public class TurnToTagCommand extends Command {
  private TagSubsystem tagSubsystem;
  private NavigationSubsystem navigationSubsystem;
  private DriveSubsystem driveSubsystem;
  private int tagID;

  public TurnToTagCommand(TagSubsystem tagSubsystem, NavigationSubsystem navigationSubsystem, DriveSubsystem driveSubsystem, int tagID) {
    this.tagSubsystem = tagSubsystem;
    this.navigationSubsystem = navigationSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(tagSubsystem, navigationSubsystem, driveSubsystem);
  }
  
  double angleToTag;
  double z;
  double x;
  double alpha;
  final double conversionToDeg = 180./Math.PI;
  final double conversionToRad = Math.PI/180.;
  RotateRobotCommand rotateRobotLeft = new RotateRobotCommand(-0.05, navigationSubsystem,driveSubsystem);
  RotateRobotCommand rotateRobotRight = new RotateRobotCommand(0.05, navigationSubsystem, driveSubsystem);

  @Override
  public void initialize() {
        TagData tagData = tagSubsystem.getAprilTag(tagID);
    if (tagData != null) { 
    x = tagData.x;
    z = tagData.z;
    alpha = tagData.alpha;
  }
}

  public double angletoTag() {
    angleToTag = Math.atan2(x, z) / Math.PI * 180;
    System.out.println(angleToTag);
    return angleToTag;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (alpha > 10 || alpha < -10) {alpha *= conversionToRad;}
    if(alpha > 0.05) {rotateRobotRight.schedule();}
    else if(alpha < -0.05) {rotateRobotLeft.schedule();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}