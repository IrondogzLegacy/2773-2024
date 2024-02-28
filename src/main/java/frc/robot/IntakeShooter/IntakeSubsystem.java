// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakeShooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Arm.RotateArmToAngleCommand;

import com.revrobotics.CANSparkMax;

public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorCANID, Constants.motorType);
  CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotor1CANID, Constants.motorType);
  CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotor2CANID, Constants.motorType);

  DigitalInput intakeIRSensor = new DigitalInput(9);

  public IntakeSubsystem() {
    intakeMotor.setSmartCurrentLimit(20);
    shooterMotor1.setSmartCurrentLimit(30);
    shooterMotor2.setSmartCurrentLimit(30);
    shooterMotor2.setInverted(false);
  }

  private NetworkTable sensorTable = NetworkTableInstance.getDefault().getTable("Sensors");
  private NetworkTableEntry infraredTableEntry = sensorTable.getEntry("IRSensor");

  public void startIntake() {
    intakeMotor.set(Constants.intakeSpeed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }

  public void startShooter() {
    shooterMotor1.set(Constants.shooterSpeed);
    shooterMotor2.set(Constants.shooterSpeed);
  }

  public void stopShooter() {
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
  }

  public void reverseIntake() {
    intakeMotor.set(Constants.reverseIntakeSpeed);
  }

  public void reverseShooter() {
    shooterMotor1.set(Constants.reverseShooterSpeed);
    shooterMotor2.set(Constants.reverseShooterSpeed);
  }

  public boolean hasRing() {return intakeIRSensor.get();}

  @Override
  public void periodic() {
    infraredTableEntry.setBoolean(hasRing());
  }

  public void runShooter(double speed) {
    shooterMotor1.set(speed);
    shooterMotor2.set(speed);
  }
}
