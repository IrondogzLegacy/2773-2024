// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;

public class TurnToTagCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private NavigationSubsystem navigationSubsystem;
  private CamSubsystem cameraSubsystem;
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("April Tag");
  private NetworkTableEntry tagIdEntry = table.getEntry("Id");

  /* Creates a new TurnToTag. */
  public TurnToTagCommand(DriveSubsystem driveSubsystem,
      NavigationSubsystem navigationSubsystem, CamSubsystem cameraSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.navigationSubsystem = navigationSubsystem;
    this.cameraSubsystem = cameraSubsystem;
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
  double distanceB;
  double distanceA;
  final double conversionToDeg = 180./Math.PI;
  final double conversionToRad = Math.PI/180.;
  double rotateSign;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    long id = tagIdEntry.getInteger(1);
        var tagData = cameraSubsystem.getAprilTag(1);
    // angle = x < 0 ? -30 : 30;
    // turnAngle = x;
    if (tagData != null) { 
    System.out.println(id);
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
    // System.out.println("Theta " + theta);
    distanceB = Math.cos(theta)*distanceToTag();
    // System.out.println("B " + distanceB);
    //drive the above distance after turning
    distanceA = Math.sin(theta)*distanceToTag();
    // System.out.println("A "+distanceA);
    //drive the above distance minus some # after the previous drive finishes
    // System.out.println("Tag One Found");
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
    long id = tagIdEntry.getInteger(1);
    var tagData = cameraSubsystem.getAprilTag(1);

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
      distanceB = Math.cos(theta)*distanceToTag();
      distanceA = Math.sin(theta)*distanceToTag();
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