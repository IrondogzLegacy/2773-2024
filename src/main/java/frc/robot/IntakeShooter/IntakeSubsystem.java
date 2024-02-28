// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakeShooter;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorCANID, Constants.motorType);
  DigitalInput intakeIRSensor = new DigitalInput(0);

  public IntakeSubsystem() {
    intakeMotor.setSmartCurrentLimit(20);  
  }

  public void startIntake() {
    intakeMotor.set(Constants.intakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void reverseIntake() {
    intakeMotor.set(Constants.reverseIntakeSpeed);
  }

  public boolean hasRing()
  {
    return intakeIRSensor.get();
  }

  @Override
  public void periodic() {
  }
}
