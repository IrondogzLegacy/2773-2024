// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class NavigationSubsystem extends SubsystemBase {
  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  public double angle;
  // Locations for the swerve drive modules relative to the robot center.
  Translation2d m_frontLeftLocation = new Translation2d(Constants.DistanceBetweenWheels, Constants.DistanceBetweenWheels);
  Translation2d m_frontRightLocation = new Translation2d(Constants.DistanceBetweenWheels, -Constants.DistanceBetweenWheels);
  Translation2d m_backLeftLocation = new Translation2d(-Constants.DistanceBetweenWheels, Constants.DistanceBetweenWheels);
  Translation2d m_backRightLocation = new Translation2d(-Constants.DistanceBetweenWheels, -Constants.DistanceBetweenWheels);

  // Creating my kinematics object using the module locations
  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
  double pitch;
  private Supplier<SwerveModulePosition[]> modulePositions;
  
  public double x;
  public double y;
  public double z;
  public double flx;
  public double fly;
  double[] fl = {flx, fly};
  public double frx;
  public double fry;
  double[] fr = {frx, fry};
  public double blx;
  public double bly;
  double[] bl = {blx, bly};
  public double brx;
  public double bry;
  double[] br = {brx, bry};
  
  double sflx;
  double sfly;
  double sfrx;
  double sfry;
  double sblx;
  double sbly;
  double sbrx;
  double sbry;

  public double fla;
  public double fra;
  public double bla;
  public double bra;


  private SwerveDriveOdometry odometry;
  private Pose2d pose;
  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem(Supplier<SwerveModulePosition[]> modulePositions) {
    this.modulePositions = modulePositions;
    Shuffleboard.getTab("Navigation").add(gyro);

   /*  Shuffleboard.getTab("Navigation").addDoubleArray("position", () -> {
      return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    });
   */
    Shuffleboard.getTab("Swerve Coordinates");

    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Left", () -> {
      return new double[] {sflx, sfly};
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Right", () -> {
      return new double[] {sfrx, sfry};
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Left", () -> {
      return new double[] {sblx, sbly};
    });
    Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Right", () -> {
      return new double[] {sbrx, sbry};
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

  public void resetOrientation() {
    gyro.reset();
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
    double fld = fl.distanceMeters;
    fla = fl.angle.getRadians();
    SwerveModulePosition fr = positions[1];
    double frd = fr.distanceMeters;
    fra = fr.angle.getRadians();
    SwerveModulePosition bl = positions[2];
    double bld = bl.distanceMeters;
    bla = bl.angle.getRadians();
    SwerveModulePosition br = positions[3];
    double brd = br.distanceMeters;
    bra = br.angle.getRadians();

    flx = fld * Math.cos(fla);
    fly = fld * Math.sin(fla);
    frx = frd * Math.cos(fra);
    fry = frd * Math.sin(fra);
    blx = bld * Math.cos(bla);
    bly = bld * Math.sin(bla);
    brx = brd * Math.cos(bra);
    bry = brd * Math.sin(bra);

    sflx += flx;
    sfly += fly;
    sfrx += frx;
    sfry += fry;
    sblx += blx;
    sbly += bly;
    sbrx += frx;
    sbry += fry;
  }

  public Double[][] getCoordinates() {
    //Returns 4 1D arrays, each representing the x and y of a module. Index 0, 1, 2, 3 represent
    //the front left, front right, back left, and back right modules respectively.
    Double[][] coordinates = {
      {sflx, sfly}, 
      {sfrx, sfry}, 
      {sblx, sbly}, 
      {sbrx, sbry}
    };
    return coordinates;
  }


}
