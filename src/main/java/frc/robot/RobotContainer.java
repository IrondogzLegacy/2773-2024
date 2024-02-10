// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  //Subsystems
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  NavigationSubsystem navigationSubsystem = new NavigationSubsystem(driveSubsystem::getPositions);
  
  //Controllers
  XboxController joy = new XboxController(0);
  
  //Commands from files
  DriveCommand driveCommand = new DriveCommand(driveSubsystem, joy, navigationSubsystem);
  CarDriveCommand carDriveCommand = new CarDriveCommand(driveSubsystem, joy);
  SwitchCommand switchCommand = new SwitchCommand(driveSubsystem, carDriveCommand, driveCommand);
  
  //Buttons
  JoystickButton button = new JoystickButton(joy, 1);
  JoystickButton switchButton = new JoystickButton(joy, 2);
  
  //Composite Commands
  
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(driveCommand);
    button.whileTrue(new RunCommand(() -> driveSubsystem.reset(), driveSubsystem));
    switchButton.onTrue(switchCommand);
  }

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

//autonomous commands
  public Command getMiddleAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  public Command getLeftAutoCommand() {
    return Commands.print("No autonomous command configured");
  }
  public Command getRightAutoCommand() {
    return Commands.print("No autonomous command configured");
  }
}
