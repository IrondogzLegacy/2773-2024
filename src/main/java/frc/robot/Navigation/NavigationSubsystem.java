// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NavigationSubsystem extends SubsystemBase {
  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  public double angle;
  // Locations for the swerve drive modules relative to the robot center.
  Translation2d frontLeftLocation = new Translation2d(-Constants.DistanceBetweenWheels/2, Constants.DistanceBetweenWheels/2);
  Translation2d frontRightLocation = new Translation2d(Constants.DistanceBetweenWheels/2, Constants.DistanceBetweenWheels/2);
  Translation2d backLeftLocation = new Translation2d(-Constants.DistanceBetweenWheels/2, -Constants.DistanceBetweenWheels/2);
  Translation2d backRightLocation = new Translation2d(Constants.DistanceBetweenWheels/2, -Constants.DistanceBetweenWheels/2);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
  double pitch;
  private Supplier<SwerveModulePosition[]> modulePositions;
  
  public double x;
  public double y;
  public double z;
  
  public double flx = 0;
  public double fly = 0;
  public double frx = 0;
  public double fry = 0;
  public double blx = 0;
  public double bly = 0;
  public double brx = 0;
  public double bry = 0;

  public double fla;
  public double fra;
  public double bla;
  public double bra;

  double[] fl = {flx, fly};
  double[] fr = {frx, fry};
  double[] bl = {blx, bly};
  double[] br = {brx, bry};

  private SwerveDriveOdometry odometry;
  public Pose2d pose;

  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem(Supplier<SwerveModulePosition[]> modulePositions) {
    this.modulePositions = modulePositions;
    Shuffleboard.getTab("Navigation").add(gyro);

    Shuffleboard.getTab("Navigation").addDoubleArray("position", () -> {
      return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    });
   
    Shuffleboard.getTab("Navigation").addDoubleArray("Robot", () -> {
      return new double[] {x, y, gyro.getDisplacementX(), gyro.getDisplacementY()};
    });
    Shuffleboard.getTab("Navigation").addDoubleArray("Front Left", () -> {
      return new double[] {flx, fly};
    });
    Shuffleboard.getTab("Navigation").addDoubleArray("Front Right", () -> {
      return new double[] {frx, fry};
    });
    Shuffleboard.getTab("Navigation").addDoubleArray("Back Left", () -> {
      return new double[] {blx, bly};
    });
    Shuffleboard.getTab("Navigation").addDoubleArray("Back Right", () -> {
      return new double[] {brx, bry};
    });

     odometry = new SwerveDriveOdometry(
      kinematics, gyro.getRotation2d(), modulePositions.get(), new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public double angle() {
    return this.angle;
  }

  public Pose2d pose() {
    return pose;
  }

  public void resetOrientation() {
    gyro.reset();
  }

  @Override
  public void periodic() {
    angle = gyro.getAngle() / 180.0 * Math.PI;
    if (angle > 2 * Math.PI) {
      angle -= 2 * Math.PI;
    } else if (angle < 0) {
      angle += 2 * Math.PI;
    }
    pitch = gyro.getPitch();
    pose = odometry.update(gyro.getRotation2d(), modulePositions.get());
    // Transform2d trans = new Transform2d(6.0, 3.20, null);
    // pose.plus(trans);
    x = pose.getX();
    y = pose.getY();
    // System.out.println(x + " , " + gyro.getDisplacementX());
    // System.out.println(y + " , " + gyro.getDisplacementY());
    // System.out.println(angle + " , " + pose.getRotation().getRadians());
    angle = pose.getRotation().getRadians();
    flx = x + (Math.cos(angle + 0.75 * Math.PI) * Constants.DistanceBetweenWheels);
    fly = y + (Math.sin(angle + 0.75 * Math.PI) * Constants.DistanceBetweenWheels);
    frx = x + (Math.cos(angle + 0.25 * Math.PI) * Constants.DistanceBetweenWheels);
    fry = y + (Math.sin(angle + 0.25 * Math.PI) * Constants.DistanceBetweenWheels);
    blx = x + (Math.cos(angle - 0.75 * Math.PI) * Constants.DistanceBetweenWheels);
    bly = y + (Math.sin(angle - 0.75 * Math.PI) * Constants.DistanceBetweenWheels);
    brx = x + (Math.cos(angle - 0.25 * Math.PI) * Constants.DistanceBetweenWheels);
    bry = y + (Math.sin(angle - 0.25 * Math.PI) * Constants.DistanceBetweenWheels);

    SwerveModulePosition[] positions = modulePositions.get();
    fla = positions[0].angle.getRadians();
    fra = positions[1].angle.getRadians();
    bla = positions[2].angle.getRadians();
    bra = positions[3].angle.getRadians();

  }

  public void reset() {
    x   = 0;
    y   = 0;
    z   = 0;
    flx = 0;
    fly = 0;
    frx = 0;
    fry = 0;
    blx = 0;
    bly = 0;
    brx = 0;
    bry = 0;
  }

  public Double[][] getCoordinates() {
    //Returns 4 1D arrays, each representing the x and y of a module. Index 0, 1, 2, 3 represent
    //the front left, front right, back left, and back right modules respectively.
    Double[][] coordinates = {
      {flx, fly}, 
      {frx, fry}, 
      {blx, bly}, 
      {brx, bry}
    };
    return coordinates;
  }


}
