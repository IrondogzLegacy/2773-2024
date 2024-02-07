// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.DriveSubsystem;

public class NavigationSubsystem extends SubsystemBase {
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private double angle;
  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(Constants.HALF_WHEEL_DISTANCE, Constants.HALF_WHEEL_DISTANCE);
  Translation2d m_frontRightLocation = new Translation2d(Constants.HALF_WHEEL_DISTANCE, -Constants.HALF_WHEEL_DISTANCE);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.HALF_WHEEL_DISTANCE, Constants.HALF_WHEEL_DISTANCE);
  Translation2d m_backRightLocation = new Translation2d(-Constants.HALF_WHEEL_DISTANCE, -Constants.HALF_WHEEL_DISTANCE);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  double pitch;
  private Supplier<SwerveModulePosition[]> modulePositions;
  
  double x;
  double y;
  double z;
  double flx;
  double fly;
  double[] fl = {flx, fly};
  double frx;
  double fry;
  double[] fr = {frx, fry};
  double blx;
  double bly;
  double[] bl = {blx, bly};
  double brx;
  double bry;
  double[] br = {brx, bry};

  private SwerveDriveOdometry odometry;
  private Pose2d pose;
  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem(Supplier<SwerveModulePosition[]> modulePositions) {
    this.modulePositions = modulePositions;
    Shuffleboard.getTab("Navigation").add(gyro);

    Shuffleboard.getTab("Navigation").addDoubleArray("position", () -> {
      return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    });

    Shuffleboard.getTab("Swerve Coordinates");

    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Left", () -> {
      return new double[] {flx, fly};
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Right", () -> {
      return new double[] {frx, fry};
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Left", () -> {
      return new double[] {blx, bly};
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Right", () -> {
      return new double[] {brx, bry};
    });

     odometry = new SwerveDriveOdometry(
      m_kinematics, gyro.getRotation2d(), modulePositions.get(), new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public double angle() {
    return this.angle;
  }


  public Pose2d pose() {
    return pose;
  }

  @Override
  public void periodic() {
    angle = gyro.getAngle() / 180.0 * Math.PI;
    pitch = gyro.getPitch();
    // pose = odometry.update(gyro.getRotation2d(), modulePositions);
    x = gyro.getDisplacementX();
    y = gyro.getDisplacementY();
    z = gyro.getDisplacementZ();

    SwerveModulePosition[] positions = modulePositions.get();
    SwerveModulePosition fl = positions[0];
    double fldistance = fl.distanceMeters;
    double flangle = fl.angle.getRadians();
    SwerveModulePosition fr = positions[1];
    double frdistance = fr.distanceMeters;
    double frangle = fr.angle.getRadians();
    SwerveModulePosition bl = positions[2];
    double bldistance = bl.distanceMeters;
    double blangle = bl.angle.getRadians();
    SwerveModulePosition br = positions[3];
    double brdistance = br.distanceMeters;
    double brangle = br.angle.getRadians();
    flx = fldistance * Math.cos(flangle);
    fly = fldistance * Math.sin(flangle);
    frx = frdistance * Math.cos(frangle);
    fry = frdistance * Math.sin(frangle);
    blx = bldistance * Math.cos(blangle);
    bly = bldistance * Math.sin(blangle);
    brx = brdistance * Math.cos(brangle);
    bry = brdistance * Math.sin(brangle);
    

    

  }
}
