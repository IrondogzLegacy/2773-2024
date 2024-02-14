// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
CANSparkMax armMotor = new CANSparkMax(Constants.armMotorCANID, Constants.motorType);
  /** Creates a new ArmSubsystem. */
  private final RelativeEncoder armRotationEncoder = armMotor.getEncoder();
  AnalogInput armPotent2 = new AnalogInput(1);

  public ArmSubsystem() {
    armMotor.setSmartCurrentLimit (30);
    armRotationEncoder.setPositionConversionFactor(3.0);

  }

  private final NetworkTable armTable = NetworkTableInstance.getDefault().getTable("Arm");
  private final NetworkTableEntry angTableEntry = armTable.getEntry("encoderAngle");
  private final NetworkTableEntry armAngleEntry = armTable.getEntry("angle");
  private final NetworkTableEntry armVoltageEntry = armTable.getEntry("voltage");

  public void rotateUp() {armMotor.set(Constants.rotationUpSpeed);}
  public void rotateDown() {armMotor.set(Constants.rotationDownSpeed);}
  public void rotateStop() {armMotor.set(0);}
  public void resetArmEncoder() {armRotationEncoder.setPosition(0);}
  public void rotate(double speed) {armMotor.set(speed);}
  public double getRotationAngle() {
    return armRotationEncoder.getPosition();
  }
  private static double map(double x, double x1, double x2, double y1, double y2) {
    return (x - x1) / (x2 - x1) * (y2 - y1) + y1;
  }
  public double getRotationAngle2() {
    return map(armPotent2.getVoltage(), Constants.ArmBottomVoltage, Constants.ArmTopVoltage, Constants.ArmMinDeg,
        Constants.ArmMaxDeg);
  }

  @Override
  public void periodic() {
    angTableEntry.setDouble(getRotationAngle());
    armVoltageEntry.setDouble(armPotent2.getVoltage());
    armAngleEntry.setDouble(armRotationEncoder.getPosition());
    // System.out.println(armEncoder.getPosition());  
  }
  public void printVoltage() {System.out.println(armPotent2.getVoltage());}
  public void printMap() {System.out.println(getRotationAngle2());}

}
