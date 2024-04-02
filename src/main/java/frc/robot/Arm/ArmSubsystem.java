// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax armMotor = new CANSparkMax(Constants.armMotorCANID, Constants.motorType);
  /** Creates a new ArmSubsystem. */
  private final RelativeEncoder armRotationEncoder = armMotor.getEncoder();

  public ArmSubsystem() {
    armMotor.setSmartCurrentLimit(35);
    armRotationEncoder.setPositionConversionFactor(4.0 / 125.0);
    armMotor.setInverted(true);

    armRotationEncoder.setPosition(1.15);
    rotateAnglePID.setSetpoint(0);
    rotateAnglePID.setTolerance(0.001);
    Shuffleboard.getTab("Navigation").addDoubleArray("Arm Speed", () -> {
      return new double[] {armMotor.get()};
    });
  }
  //first array is with a high charge battery, second is with a low charge battery
  public double[][] speakerAngles = {{0.25,0.31,0.35,0.37,0.39,0.41,0.43,0.45},{0.31,0.362,0.39,0.435,0.465,0.49,0.49,0.5}};

  private final NetworkTable armTable = NetworkTableInstance.getDefault().getTable("Arm");
  private final NetworkTableEntry angTableEntry = armTable.getEntry("encoderAngle");
  private final NetworkTableEntry armAngleEntry = armTable.getEntry("angle");

  public double angle() {
    return rotateAnglePID.getSetpoint();
  }

  private boolean overrideSafety = false;

  public void noSafety() {
    overrideSafety = true;
  }

  public void setAngle(double angle) {
    if (!overrideSafety) {
      angle = MathUtil.clamp(angle, Constants.ArmMinDeg, Constants.ArmMaxDeg);
    }
    rotateAnglePID.setSetpoint(angle);
  }

  public void rotateBy(double delta) {
    setAngle(angle() + delta);
  }

  public void rotateUp() {
    rotateBy(0.02);
  }

  public void rotateDown() {
    rotateBy(-0.02);
  }

  public void rotateStop() {
    setAngle(getRotationAngle());
  }

  public void resetArmEncoder() {
    rotateAnglePID.setSetpoint(0);
    armRotationEncoder.setPosition(0);
  }

  public double getRotationAngle() {
    return armRotationEncoder.getPosition();
  }

  public PIDController rotateAnglePID = new PIDController(5, 9, 0);
// 5, 9
  @Override
  public void periodic() {
    angTableEntry.setDouble(getRotationAngle());
    armAngleEntry.setDouble(angle());
    overrideSafety = false;

    double rotate = rotateAnglePID.calculate(getRotationAngle());
    if (rotateAnglePID.atSetpoint()) {

      armMotor.stopMotor();
      return;
    }
    rotate = MathUtil.clamp(rotate, -Constants.ArmMaxRotationSpeed, Constants.ArmMaxRotationSpeed);
    armMotor.set(rotate);
  }

  public boolean atSetpoint() {
    return rotateAnglePID.atSetpoint();
  }

}
