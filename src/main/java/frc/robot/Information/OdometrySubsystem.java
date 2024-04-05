// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  public double[] setX = {0.0, 0.0, 0.0, 0.0, 0.0};
  public double[] setY = {0.0, 0.0, 0.0, 0.0, 0.0};
  public int i = 0;

  private NavigationSubsystem navSub;

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

public void addSetPoint() {
    setX[i] = x;
    setY[i] = y;
    i++;
}
}
