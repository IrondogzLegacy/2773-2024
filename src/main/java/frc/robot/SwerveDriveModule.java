// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class SwerveDriveModule {
  private static final double ROTATION_LIMIT_SPEED = 0.7;

  public CANSparkMax driveMotor;
  public CANSparkMax rotateMotor;
  public CANcoder encoder;
  public RelativeEncoder distanceEncoder;
  public RelativeEncoder rotationEncoder;
  public int id;
  public double alpha;
  private PIDController pidRotate;

  public SwerveDriveModule(int driveId, int rotateId, int encoderId, double alpha) {
    driveMotor = new CANSparkMax(driveId, Constants.motorType);
    rotateMotor = new CANSparkMax(rotateId, Constants.motorType);
    encoder = new CANcoder(encoderId);
    distanceEncoder = driveMotor.getEncoder();
    distanceEncoder.setPositionConversionFactor((1/6.75) * 2 * Math.PI * 4 * 0.0254);
    rotationEncoder = rotateMotor.getEncoder();
    rotationEncoder.setPositionConversionFactor((1.0/21) * 2 * Math.PI);
    rotationEncoder.setPosition(position());
    id = encoderId;
    this.alpha = alpha;
    this.pidRotate = new PIDController(0.63, 0, 0);
  }
  
  public void drive(double speed, double rotate) {
    driveMotor.set(speed);
    rotateMotor.set(rotate);
    System.out.println(id + " " + distanceEncoder.getPosition());
  }

  public SwerveModulePosition getMotorEncoderPosition() {
    return new SwerveModulePosition(
        distanceEncoder.getPosition(), new Rotation2d(-rotationEncoder.getPosition()));
  }
  
  public void directionalDrive(double speed, double angle) {
    pidRotate.setSetpoint(0);
    double pos = -position() - angle;
    while (pos < -Math.PI)
      pos += 2 * Math.PI;
    while (pos >= Math.PI)
      pos -= 2 * Math.PI;
    // -PI =< pos < PI

    double direction = 1.0;
    if (pos < -Math.PI / 2) {
      direction = -1.0;
      pos += Math.PI;
    }
    if (pos > Math.PI / 2) {
      direction = -1.0;
      pos -= Math.PI;
    }
    double speedOfRotation = pidRotate.calculate(pos);
    speedOfRotation = MathUtil.clamp(speedOfRotation, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(speedOfRotation);

    driveMotor.set(speed * direction);
  }

  public double position() {
    // position [-0.5..0.5)
    double value = encoder.getAbsolutePosition().getValueAsDouble() - alpha;
    if (value < -0.5)
      value += 1.0;
    if (value >= 0.5)
      value -= 1.0;
    return value * 2 * Math.PI;
  }

  public double rawPosition() {
    return encoder.getAbsolutePosition().getValueAsDouble();
  }

  public void reset() {
    pidRotate.setSetpoint(0);
    double s = pidRotate.calculate(-position());
    s = MathUtil.clamp(s, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(s);
  }

  public void stop() {
    driveMotor.stopMotor();
    rotateMotor.stopMotor();
  }
}
