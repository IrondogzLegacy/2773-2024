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
  Translation2d frontLeftLocation = new Translation2d(-Constants.DistanceBetweenWheels, Constants.DistanceBetweenWheels);
  Translation2d frontRightLocation = new Translation2d(Constants.DistanceBetweenWheels, Constants.DistanceBetweenWheels);
  Translation2d backLeftLocation = new Translation2d(-Constants.DistanceBetweenWheels, -Constants.DistanceBetweenWheels);
  Translation2d backRightLocation = new Translation2d(Constants.DistanceBetweenWheels, -Constants.DistanceBetweenWheels);

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

  double[] fl = {flx, fly};
  double[] fr = {frx, fry};
  double[] bl = {blx, bly};
  double[] br = {brx, bry};

  public double fla;
  public double fra;
  public double bla;
  public double bra;

  double fld;
  double frd;
  double bld;
  double brd;


  private SwerveDriveOdometry odometry;
  private Pose2d pose;
  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem(Supplier<SwerveModulePosition[]> modulePositions) {
    this.modulePositions = modulePositions;
    Shuffleboard.getTab("Navigation").add(gyro);

    Shuffleboard.getTab("Navigation").addDoubleArray("position", () -> {
      return new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    });
   
    // Shuffleboard.getTab("Swerve Coordinates");
    // Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Robot", () -> {
    //   return new double[] {x, y};
    // });
    // Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Left", () -> {
    //   return new double[] {flx, fly};
    // });
    // Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Front Right", () -> {
    //   return new double[] {frx, fry};
    // });
    // Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Left", () -> {
    //   return new double[] {blx, bly};
    // });
    // Shuffleboard.getTab("Swerve Coordinates").addDoubleArray("Back Right", () -> {
    //   return new double[] {brx, bry};
    // });

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
    pitch = gyro.getPitch();
    pose = odometry.update(gyro.getRotation2d(), modulePositions.get());
    x = gyro.getDisplacementX();
    y = gyro.getDisplacementY();
    z = gyro.getDisplacementZ();

    // SwerveModulePosition[] positions = modulePositions.get();

    // SwerveModulePosition fl = positions[0];
    // fld = fl.distanceMeters;
    // fla = fl.angle.getRadians();
    // SwerveModulePosition fr = positions[1];
    // frd = fr.distanceMeters;
    // fra = fr.angle.getRadians();
    // SwerveModulePosition bl = positions[2];
    // bld = bl.distanceMeters;
    // bla = bl.angle.getRadians();
    // SwerveModulePosition br = positions[3];
    // brd = br.distanceMeters;
    // bra = br.angle.getRadians();

    

    // // System.out.println(fld);

    // flx = fld * Math.cos(fla);
    // fly = fld * Math.sin(fla);
    // frx = frd * Math.cos(fra);
    // fry = frd * Math.sin(fra);
    // blx = bld * Math.cos(bla);
    // bly = bld * Math.sin(bla);
    // brx = brd * Math.cos(bra);
    // bry = brd * Math.sin(bra);

    // sflx += flx;
    // sfly += fly;
    // sfrx += frx;
    // sfry += fry;
    // sblx += blx;
    // sbly += bly;
    // sbrx += frx;
    // sbry += fry;

    x = pose.getX();
    y = pose.getY();
    angle = pose.getRotation().getRadians();
    flx = x + (Math.cos(angle) * Constants.DistanceBetweenWheels);
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
