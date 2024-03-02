// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakeShooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotor1CANID, Constants.motorType);
    CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotor2CANID, Constants.motorType);  
    
    public ShooterSubsystem() {
    shooterMotor1.setSmartCurrentLimit(30);
    shooterMotor2.setSmartCurrentLimit(30);
    shooterMotor2.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startShooter() {
    shooterMotor1.set(Constants.shooterSpeed);
    shooterMotor2.set(Constants.shooterSpeed);
  }

  public void startShooterLow()
  {
    shooterMotor1.set(Constants.shooterLowSpeed);
    shooterMotor2.set(Constants.shooterLowSpeed);
  }

  public void stopShooter() {
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
  }

  public void reverseShooter() {
    shooterMotor1.set(Constants.reverseShooterSpeed);
    shooterMotor2.set(Constants.reverseShooterSpeed);
  }

  public void runShooter(double speed) {
    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }

}
