// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  public SwerveDriveModule blMotor = new SwerveDriveModule(Constants.backLeftModuleDriveCANID, Constants.backLeftModuleRotateCANID, Constants.backLeftModuleEncoderCANID, 0.3686);
  public SwerveDriveModule brMotor = new SwerveDriveModule(Constants.backRightModuleDriveCANID, Constants.backRightModuleRotateCANID, Constants.backRightModuleEncoderCANID, -0.1597);
  public SwerveDriveModule frMotor = new SwerveDriveModule(Constants.frontRightModuleDriveCANID, Constants.frontRightModuleRotateCANID, Constants.frontRightModuleEncoderCANID, -0.48657);
  public SwerveDriveModule flMotor = new SwerveDriveModule(Constants.frontLeftModuleDriveCANID, Constants.frontLeftModuleRotateCANID, Constants.frontLeftModuleEncoderCANID, 0.35522);

  public SwerveModulePosition[] getPositions()
  {
    return new SwerveModulePosition[] {
    flMotor.getMotorEncoderPosition(), frMotor.getMotorEncoderPosition(),
    blMotor.getMotorEncoderPosition(), brMotor.getMotorEncoderPosition()
  };
  }

  /** Creates a new TestSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // System.out.println("BL:" + blMotor.rawPosition());
    // System.out.println("BR:" + brMotor.rawPosition());
    // System.out.println("FL:" + flMotor.rawPosition());
    // System.out.println("FR:" + frMotor.rawPosition());

    }

  public void drive(double speed, double rotate) {
    blMotor.drive(speed, rotate);
    brMotor.drive(speed, rotate);
    frMotor.drive(speed, rotate);
    flMotor.drive(speed, rotate);
  }

  public void directionalDrive(double speed, double angle) {
    blMotor.directionalDrive(speed, angle);
    brMotor.directionalDrive(speed, angle);
    frMotor.directionalDrive(speed, angle);
    flMotor.directionalDrive(speed, angle);
  }

  static class Vec {
    double phi;
    double r;
    Vec(double r, double phi) {
      this.phi = phi; this.r = r;
    }
    Vec add(Vec a) {
      double x = this.r * Math.cos(this.phi);
      double y = this.r * Math.sin(this.phi);
      x += a.r * Math.cos(a.phi);
      y += a.r * Math.sin(a.phi);
      return new Vec(
        Math.sqrt(x * x + y * y),
        Math.atan2(y, x)
      );
    }
  }

  public void directionalDrive(double speed, double angle, double rotation) {
    Vec bl = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
    Vec br = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
    Vec fr = new Vec(speed, angle).add(new Vec(rotation, Math.PI / 4));
    Vec fl = new Vec(speed, angle).add(new Vec(rotation, -Math.PI / 4));
    blMotor.directionalDrive(bl.r, bl.phi);
    brMotor.directionalDrive(br.r, br.phi);
    frMotor.directionalDrive(fr.r, fr.phi);
    flMotor.directionalDrive(fl.r, fl.phi);
  }

  public void resetMotors() {
    blMotor.reset();
    brMotor.reset();
    frMotor.reset();
    flMotor.reset();
  }

  public void stop() {
    blMotor.stop();
    brMotor.stop();
    frMotor.stop();
    flMotor.stop();
  }

  public void rotate(double speed) {
    frMotor.directionalDrive(speed, Math.PI / 4);
    brMotor.directionalDrive(speed, 3 * Math.PI / 4);
    blMotor.directionalDrive(speed, -3 * Math.PI / 4);
    flMotor.directionalDrive(speed, -Math.PI / 4);
  }

  public void carDrive(double rotationFactor, double speed) {
    final double HALF_WHEEL_DISTANCE = Constants.DistanceBetweenWheels;
    double distance = 1 / (rotationFactor + 1e-7);

    speed *= Math.copySign(1, distance);

    double rl = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance - HALF_WHEEL_DISTANCE) * (distance - HALF_WHEEL_DISTANCE));
    double rr = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance + HALF_WHEEL_DISTANCE) * (distance + HALF_WHEEL_DISTANCE));

    double flAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double frAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double blAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);
    double brAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);

    double kl = 1, kr = 1;
    if (distance < 0) {
      kl = rr / rl;
    } else {
      kr = rl / rr;
    }

    frMotor.directionalDrive(kr*speed, frAngle);
    brMotor.directionalDrive(kr*speed, brAngle);
    blMotor.directionalDrive(kl*speed, blAngle);
    flMotor.directionalDrive(kl*speed, flAngle);
  }

  public void rotateTo(double radians) {
    
  }
}
