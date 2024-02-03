// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class NavigationSubsystem extends SubsystemBase {
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private double angle;

  SwerveDriveKinematics kinematics = new SwerveDriveKinematics();
  SwerveModulePosition[] positions;
  SwerveDriveWheelPositions modulePositions = new SwerveDriveWheelPositions(positions);
  Rotation2d gyroRotation2d = gyro.getRotation2d();
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyroRotation2d, modulePositions.positions);
  Pose2d pose;
  double pitch;
  double x;
  double y;
  double z;
  double fly;
  double flx;
  double fry;
  double frx;
  double bly;
  double blx;
  double bry;
  double brx;
  double HALF_WHEEL_DISTANCE = 0.5207;
  double hypo = Math.sqrt(HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE + HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE);
  
  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem() {
    Shuffleboard.getTab("Navigation").add(gyro);
  }

  public double angle() {
    return this.angle;
  }

  @Override
  public void periodic() {
    angle = gyro.getAngle() / 180.0 * Math.PI;
    pitch = gyro.getPitch();
    pose = odometry.update(gyro.getRotation2d(), modulePositions);
    x = gyro.getDisplacementX();
    y = gyro.getDisplacementY();
    z = gyro.getDisplacementZ();
    frx = hypo * Math.cos(angle + 45);
    fry = hypo * Math.sin(angle + 45);
    flx = hypo * Math.cos(angle + 135);
    fly = hypo * Math.sin(angle + 135);
    brx = hypo * Math.cos(angle + 225);
    bry = hypo * Math.sin(angle + 225);
    blx = hypo * Math.cos(angle + 315);
    bly = hypo * Math.sin(angle + 315);
  }
}
