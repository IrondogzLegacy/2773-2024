// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

public final class Constants {
    // The distance between swerve wheels
    public static final double DistanceBetweenWheels = 0.616;
    //MotorType of our Neos & PWMs
    public static final MotorType motorType = MotorType.kBrushless;
    //CANIDs for Intake, Shooter, Arm
    public static final int intakeMotorCANID = 18;
    public static final int shooterMotor1CANID = 13;
    public static final int shooterMotor2CANID = 15; 
    public static final int armMotorCANID = 24;
    //CANIDs for Swerve Modules
        //Drive Motors
    public static final int frontLeftModuleDriveCANID = 17;
    public static final int frontRightModuleDriveCANID = 19;
    public static final int backLeftModuleDriveCANID = 10;
    public static final int backRightModuleDriveCANID = 22;
        //Rotation Motors
    public static final int frontLeftModuleRotateCANID = 16;
    public static final int frontRightModuleRotateCANID = 12;
    public static final int backLeftModuleRotateCANID = 11;
    public static final int backRightModuleRotateCANID = 23;    
        //CTRE CANCoders (Encoders)
    public static final int frontLeftModuleEncoderCANID = 52;
    public static final int frontRightModuleEncoderCANID = 53;
    public static final int backLeftModuleEncoderCANID = 54;
    public static final int backRightModuleEncoderCANID = 55;

    public static final double DriveSpeedMultiplier = 0.7;
    public static final double RotateSpeedMultiplier = 0.3;

    //Speeds for Intake, Shooter, Arm
    public static final double intakeSpeed = 0.8;
    public static final double shooterSpeed = 1.0;
    public static final double shooterLowSpeed = 0.3;
    public static final double rotationUpSpeed = 0.2;
    public static final double rotationDownSpeed = -0.1;
    public static final double reverseIntakeSpeed = -0.4;
    public static final double reverseShooterSpeed = -0.2;
    
    public static final double WheelRotateSpeed = 0.5;
    
    public static final double ControlArmSpeed = 0.01;

    //Constants for ArmControlCommand
    public static final double ArmMaxDeg = 1.1;
    public static final double ArmMinDeg = 0;
    public static final double armMaxRotationOverride = 0.1;
    public static final double ControllerDeadzone = 0.1;
    public static final double ArmMaxRotationSpeed = 0.3;
    
    //Digital Ports
    public static final int IRSensorPort = 9;

    //Climbing Speeds & Constants
        //CANID for Climbing motor
    public static final int climbMotorCANID = 20;
    public static final double climbSpeed = 0.9;
    public static final double letGoSpeed = -0.3;
    
    //Distances (meters)
    public static final double betweenInsideNotes = 57 * 0.0254;
    public static final double betweenMiddleStartAndInsideNote = 65.08 * 0.0254;
    public static final double betweenSideAwayAndNote = 0 * 0.0254;
    public static final double extraIntakeNeeded = 0 * 0.0254;
    public static final double betweenOutsideNotes = 66 * 0.0254;
    public static final double initialAngledDistance = 0 * 0.0254;
    public static final double initialDistanceToNote = 0 * 0.0254;

    public static final double outsideAutoInit1 = 18.686 * 0.0254;
    public static final double outsideAutoInit2 = 18.686 * 0.0254;
    public static final double outsideAutoInit3 = 295.96 * 0.0254;
    public static final double outsideAutoInit4 = 70.37  * 0.0254;

    //Angles for speaker shooting
    public static final double sideShootAngle   = 0.3; //Needs to be double checked
    public static final double middleShootAngle = 0.308;
}