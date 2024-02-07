// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private static final String kMiddleAuto = "Middle Position";
  private static final String kLeftAuto = "Left Position";
  private static final String kRightAuto = "Right Position";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_chooser.setDefaultOption("Middle Position", kMiddleAuto);
    m_chooser.addOption("Left Position", kLeftAuto);
    m_chooser.addOption("Right Position", kRightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    SmartDashboard.putData(m_chooser);
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
/*
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    } */
  }

  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kLeftAuto:
        m_autonomousCommand = m_robotContainer.getLeftAutoCommand();
        break;
      case kRightAuto:
        m_autonomousCommand = m_robotContainer.getRightAutoCommand();
      case kMiddleAuto:
      default:
        m_autonomousCommand = m_robotContainer.getMiddleAutonomousCommand();
        break;
    }
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
