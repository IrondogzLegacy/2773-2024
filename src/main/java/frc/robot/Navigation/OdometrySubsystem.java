// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class OdometrySubsystem extends SubsystemBase {

  public OdometrySubsystem(NavigationSubsystem navSub) {
    this.navSub = navSub;
    Shuffleboard.getTab("Navigation").addDoubleArray("Robot", () -> {
      return new double[] {x, y};
    });
  }

  public double x;
  public double y;
  public double displacementX;
  public double displacementY;
  public double angle;

  private NavigationSubsystem navSub;
  // public PowerDistribution powerHub = new PowerDistribution(Constants.PowerDistributionHubCANID, Constants.PowerHubModuleType);


  @Override
  public void periodic() {
    x += navSub.displacementX;
    y += navSub.displacementY;
    displacementX = navSub.displacementX;
    displacementY = navSub.displacementY;
    angle = navSub.angle;
    //System.out.println(x + " , "+  y);
  }

  public void setPosition(double gX, double gY) {
    x = gX;
    y = gY;
  }
}
