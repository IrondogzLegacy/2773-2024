// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

public final class Constants {
    // Half of the distance between swerve wheels
    public static final double DISTANCE_BTW_WHEELS = 0.6;
    public static final MotorType motorType = MotorType.kBrushless;
    public static final int intakeMotorCANID = 18;
    public static final int shooterMotor1CANID = 20;
    public static final int shooterMotor2CANID = 24; 
    public static final double intakeSpeed = 0.4;
    public static final double shooterSpeed = 1.0;
    public static final int armMotorCANID = 0;
    public static final double rotationUpSpeed = 0.2;
    public static final double rotationDownSpeed = -0.1;
    public static final double WHEEL_ROTATE_SPEED = 0.5;
    
    public static final double ArmMaxDeg = 0;
    public static final double ArmBottomVoltage = 0;
    public static final double ArmTopVoltage = 0;
    public static final double ArmMinDeg = 0;
    public static final double armMaxRotationOverride = 0;
    public static final double armMaxAngle = 0;
    public static final double armMaxPositionOverride = 0;
    public static final double armMaxPosition = 0;
    public static final double ControllerDeadzone = 0;
    public static final int ArmMaxRotationSpeed = 0;
    
}
