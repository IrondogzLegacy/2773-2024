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

  private static final String krMiddleAuto = "Red Middle Position";
  private static final String kbMiddleAuto = "Blue Middle Position";
  private static final String krLeftAuto = "Red Left Position";
  private static final String kbLeftAuto = "Blue Left Position";
  private static final String krRightAuto = "Red Right Position";
  private static final String kbRightAuto = "Blue Right Position";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_chooser.setDefaultOption("Red Middle Position", krMiddleAuto);
    m_chooser.addOption("Blue Middle Position", kbMiddleAuto);
    m_chooser.addOption("Red Left Position", krLeftAuto);
    m_chooser.addOption("Red Right Position", krRightAuto);
    m_chooser.addOption("Blue Right Position", kbRightAuto);
    m_chooser.addOption("Blue Left Position", kbLeftAuto);
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
        // m_autonomousCommand = m_robotContainer.getRedLeftAutoCommand();
        // m_autonomousCommand = m_robotContainer.getRedRightAutoCommand();
        // m_autonomousCommand = m_robotContainer.getRedLeftAutoCommand();
        // m_autonomousCommand = m_robotContainer.getRedLeftAutoCommand();
        // m_autonomousCommand = m_robotContainer.getRedMiddleAutonomousCommand();
        // m_autonomousCommand = m_robotContainer.getBlueMiddleAutonomousCommand();
        // m_autonomousCommand = m_robotContainer.getRedMiddleAutonomousCommand();
        m_autonomousCommand = m_robotContainer.middleAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
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
