// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel.MotorType;

public final class Constants {
    // Half of the distance between swerve wheels
    public static final double HALF_WHEEL_DISTANCE = 0.5207;
    public static final MotorType motorType = MotorType.kBrushless;
    public static final int intakeMotorCANID = 18;
    public static final int shooterMotor1CANID = 15;
    public static final int shooterMotor2CANID = 13; 
    public static final double intakeSpeed = 0.3;
    public static final double shooterSpeed = 0.3;
    
}
