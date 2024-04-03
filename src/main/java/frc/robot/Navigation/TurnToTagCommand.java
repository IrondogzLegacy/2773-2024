// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;

public class TurnToTagCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private TagSubsystem tagSubsystem;
  private int tagID;

  /* Creates a new TurnToTag. */
  public TurnToTagCommand(DriveSubsystem driveSubsystem, TagSubsystem tagSubsystem, int tagID) {
    this.driveSubsystem = driveSubsystem;
    this.tagSubsystem = tagSubsystem;
  }
  double turnAngle;
  double angleToTag;
  double dis;
  double z;
  double x;
  double y;
  double sinAlpha;
  double minusCosAlpha;
  double tagRotation;
  double fullTurnAngle;
  double theta;
  final double conversionToDeg = 180./Math.PI;
  final double conversionToRad = Math.PI/180.;
  double rotateSign;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        var tagData = tagSubsystem.getAprilTag(tagID);
    // angle = x < 0 ? -30 : 30;
    // turnAngle = x;
    if (tagData != null) { 
    x = tagData.x;
    z = tagData.z;
    tagRotation = tagData.alpha;
    System.out.println("Alpha " + tagRotation);
    if (tagRotation > 0) {fullTurnAngle = 90-tagRotation*conversionToDeg;}
    if (tagRotation < 0) {fullTurnAngle = -90-tagRotation*conversionToDeg;}

    if (fullTurnAngle > 0) {rotateSign = -90;}
    if (fullTurnAngle < 0) {rotateSign = 90;}
    // System.out.println("Beta " + fullTurnAngle);
    //turn the above angle
    theta = (fullTurnAngle - angleToTag)*conversionToRad;
   
    /*RotationCommand rotationCommand = new RotationCommand(driveSubsystem, navigationSubsystem, fullTurnAngle);
    MoveDistanceCommand moveDistanceB = new MoveDistanceCommand(driveSubsystem, navigationSubsystem, distanceB);
    MoveDistanceCommand moveDistanceA = new MoveDistanceCommand(driveSubsystem, navigationSubsystem, distanceA-1);
    RotationCommand rotate90 = new RotationCommand(driveSubsystem, navigationSubsystem, rotateSign);*/

    //rotationCommand.andThen(moveDistanceB).andThen(rotate90).andThen(moveDistanceA).schedule();
    //rotationCommand.schedule();*/
  }
}

  public double distanceToTag() {
    var distanceToTag = (Math.sqrt(x * x + z * z) * 3.28);
    // For movement, the robot will turn angleToTag, and then move distanceToTag
    System.out.println(x);
    return distanceToTag;
  }

  public double angletoTag() {
    angleToTag = Math.atan2(x, z) / Math.PI * 180;
    System.out.println(angleToTag);
    return angleToTag;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var tagData = tagSubsystem.getAprilTag(1);

    if (tagData != null) { 
      x = tagData.x;
      y = tagData.y;
      z = tagData.z;
      tagRotation = tagData.alpha;
      System.out.println("Alpha " + tagRotation);
      if (tagRotation > 0) {fullTurnAngle = 90-tagRotation*conversionToDeg;}
      if (tagRotation < 0) {fullTurnAngle = -90-tagRotation*conversionToDeg;}

      if (fullTurnAngle > 0) {rotateSign = -90;}
      if (fullTurnAngle < 0) {rotateSign = 90;}
      theta = (fullTurnAngle - angleToTag)*conversionToRad;
    }
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