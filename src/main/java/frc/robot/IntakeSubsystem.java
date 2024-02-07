// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;


public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorCANID, Constants.motorType);
  CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotor1CANID, Constants.motorType);
  CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotor2CANID, Constants.motorType);

  public IntakeSubsystem() {
    intakeMotor.setSmartCurrentLimit (20);
    shooterMotor1.setSmartCurrentLimit(20);
    shooterMotor2.setSmartCurrentLimit(20);
  }

  public void startIntake() {intakeMotor.set(Constants.intakeSpeed);}
  public void stopIntake() {intakeMotor.stopMotor();}
  public void startShooter() {shooterMotor1.set(Constants.shooterSpeed);}
  public void stopShooter() {shooterMotor1.stopMotor();}

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
