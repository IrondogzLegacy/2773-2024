// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

public final class Constants {
    // The distance between swerve wheels
    public static final double DISTANCE_BTW_WHEELS = 0.6;
    //MotorType of our Neos & PWMs
    public static final MotorType motorType = MotorType.kBrushless;
    //CANIDs for Intake, Shooter, Arm
    public static final int intakeMotorCANID = 18;
    public static final int shooterMotor1CANID = 13;
    public static final int shooterMotor2CANID = 15; 
    public static final int armMotorCANID = 24;
    //CANIDs for Swerve Modules
        //Drive Motors
    public static final int frontLeftModuleDriveCANID = 19;
    public static final int frontRightModuleDriveCANID = 22;
    public static final int backLeftModuleDriveCANID = 17;
    public static final int backRightModuleDriveCANID = 10;
        //Rotation Motors
    public static final int frontLeftModuleRotateCANID = 12;
    public static final int frontRightModuleRotateCANID = 23;
    public static final int backLeftModuleRotateCANID = 16;
    public static final int backRightModuleRotateCANID = 11;    
        //CTRE CANCoders (Encoders)
    public static final int frontLeftModuleEncoderCANID = 53;
    public static final int frontRightModuleEncoderCANID = 55;
    public static final int backLeftModuleEncoderCANID = 52;
    public static final int backRightModuleEncoderCANID = 54;

    //Speeds for Intake, Shooter, Arm
    public static final double intakeSpeed = 0.4;
    public static final double shooterSpeed = 1.0;
    public static final double rotationUpSpeed = 0.2;
    public static final double rotationDownSpeed = -0.1;
    public static final double reverseIntakeSpeed = -0.2;
    public static final double reverseShooterSpeed = -0.2;
    
    public static final double WHEEL_ROTATE_SPEED = 0.5;
    
    //Constants for ArmControlCommand
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
    
    
    //Offsets for GoToTagCommand
}
