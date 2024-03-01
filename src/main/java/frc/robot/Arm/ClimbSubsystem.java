// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  CANSparkMax climbingMotor = new CANSparkMax(Constants.climbMotorCANID, Constants.motorType);
  private final RelativeEncoder climbEncoder = climbingMotor.getEncoder();
  public boolean climbOverride = false;
  private final XboxController armStick;


  public ClimbSubsystem(XboxController armStick) {
    this.armStick = armStick;
    climbEncoder.setPositionConversionFactor(1.0);
  }

  @Override
  public void periodic() {
    if(armStick.getRawButton(7)) {climbOverride = true;}
    if(climbEncoder.getPosition() < 0 || !climbOverride) {climbStop();}
  }

  public void climb() {climbingMotor.set(Constants.climbSpeed);}
  public void letGo() {climbingMotor.set(Constants.letGoSpeed);}
  public void climbStop() {climbingMotor.set(0);}
  public void resetClimbEncoder() {climbEncoder.setPosition(0);}


}
