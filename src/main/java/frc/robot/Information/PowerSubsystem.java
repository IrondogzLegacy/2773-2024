// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowerSubsystem extends SubsystemBase {
  private PowerDistribution powerHub = new PowerDistribution(Constants.PowerDistributionHubCANID, Constants.PowerHubModuleType);

  public PowerSubsystem() {}
  public static double currentVoltage;
  public static double currentCurrent;
  public static double currentTemp;
  
  @Override
  public void periodic() {
  currentVoltage = powerHub.getVoltage();
  currentCurrent = powerHub.getTotalCurrent();
  currentTemp = powerHub.getTemperature();
  }
}
