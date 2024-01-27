// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  SwerveDriveModule blMotor = new SwerveDriveModule(17, 21, 52, 0.111083984375);
  SwerveDriveModule brMotor = new SwerveDriveModule(10, 11, 54, 0.13037109375);
  SwerveDriveModule frMotor = new SwerveDriveModule(22, 23, 55,  -0.43115234375);
  SwerveDriveModule flMotor = new SwerveDriveModule(19, 12, 53, 0.244873046875);
  
  /** Creates a new TestSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
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

  public void reset() {
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
    frMotor.directionalDrive(speed, Math.PI/4);
    brMotor.directionalDrive(speed, 3*Math.PI/4);
    blMotor.directionalDrive(speed, -3*Math.PI/4);
    flMotor.directionalDrive(speed, -Math.PI/4);
  }
}
