// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem extends SubsystemBase {
  NavigationSubsystem navigationSubsystem;
  
  
  public OdometrySubsystem(NavigationSubsystem navigationSubsystem) {
    this.navigationSubsystem = navigationSubsystem;


  }

  @Override
  public void periodic() {
    
  
  }

  public void calculateRobotPosition()
  {

  }

  public void calculuateModulePosition(SwerveModule module)
  {
       
  }
}
