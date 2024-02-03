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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class NavigationSubsystem extends SubsystemBase {
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private double angle;

  // SwerveDriveKinematics kinematics = new SwerveDriveKinematics();
  // SwerveModulePosition[] positions;
  // SwerveDriveWheelPositions modulePositions = new
  // SwerveDriveWheelPositions(positions);
  // Rotation2d gyroRotation2d = gyro.getRotation2d();
  // SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
  // gyroRotation2d, modulePositions.positions);
  // Pose2d pose;
  double pitch;
  double x;
  double y;
  double z;

  // double hypo = Math.sqrt(Constants.HALF_WHEEL_DISTANCE *
  // Constants.HALF_WHEEL_DISTANCE + Constants.HALF_WHEEL_DISTANCE *
  // Constants.HALF_WHEEL_DISTANCE);

  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem() {
    Shuffleboard.getTab("Navigation").add(gyro);

    Shuffleboard.getTab("Navigation").addDoubleArray("position", () -> {
      return new double[] { x, y };
    });
  }

  public double angle() {
    return this.angle;
  }

  public Pose3d pose3d() {
    return new Pose3d(x, y, z, new Rotation3d(0, 0, angle));
  }

  public Pose2d pose() {
    return new Pose2d(x, y, new Rotation2d(angle));
  }

  @Override
  public void periodic() {
    angle = gyro.getAngle() / 180.0 * Math.PI;
    pitch = gyro.getPitch();
    // pose = odometry.update(gyro.getRotation2d(), modulePositions);
    x = gyro.getDisplacementX();
    y = gyro.getDisplacementY();
    z = gyro.getDisplacementZ();

  }
}
