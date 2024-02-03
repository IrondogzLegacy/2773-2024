// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavigationSubsystem extends SubsystemBase {

  AHRS gyro = new AHRS(SPI.Port.kMXP);
  /** Creates a new NavigationSubsystem. */
  public NavigationSubsystem() {}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
