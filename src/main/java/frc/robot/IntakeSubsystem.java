// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import com.revrobotics.CANSparkMax;


public class IntakeSubsystem extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorCANID, Constants.motorType);
  CANSparkMax shooterMotor1 = new CANSparkMax(Constants.shooterMotor1CANID, Constants.motorType);
  CANSparkMax shooterMotor2 = new CANSparkMax(Constants.shooterMotor2CANID, Constants.motorType);

  public IntakeSubsystem() {
    intakeMotor.setSmartCurrentLimit (20);
    shooterMotor1.setSmartCurrentLimit(20);
    shooterMotor2.setSmartCurrentLimit(20);
    shooterMotor2.setInverted(true);
  }

  public void startIntake() {intakeMotor.set(Constants.intakeSpeed);}
  public void stopIntake() {intakeMotor.stopMotor();}
  public void startShooter() {shooterMotor1.set(Constants.shooterSpeed); shooterMotor2.set(Constants.shooterSpeed);}
  public void stopShooter() {shooterMotor1.stopMotor(); shooterMotor2.stopMotor();}

  /*InstantCommand startIntaking = new InstantCommand(IntakeSubsystem::startIntake);
  public Command fullIntake() {
    return null;
    return ParallelRaceGroup(new WaitCommand(8));
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
