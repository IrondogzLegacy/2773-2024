// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Navigation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem extends SubsystemBase {
  /** Creates a new OdometrySubsystem. */
  public OdometrySubsystem(NavigationSubsystem navSub) {
    this.navSub = navSub;
  }

  public double x;
  public double y;
  public double angle;

  private NavigationSubsystem navSub;

  @Override
  public void periodic() {
    x += navSub.displacementX;
    y += navSub.displacementY;
    angle = navSub.angle;
  }

  public void setPosition(double gX, double gY) {
    x = gX;
    y = gY;
  }
}
