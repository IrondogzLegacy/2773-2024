// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants;


public class ClimbSubsystem extends SubsystemBase {
  private final CANSparkMax climbingMotor = new CANSparkMax(Constants.climbMotorCANID, Constants.motorType);
  private final XboxController armStick;

  public ClimbSubsystem(XboxController armStick) {
    this.armStick = armStick;
  }

  @Override
  public void periodic() {
  }

  public void climb() {climbingMotor.set(Constants.climbSpeed);}
  public void letGo() {climbingMotor.set(Constants.letGoSpeed);}
  public void climbStop() {climbingMotor.set(0);}

  

}
