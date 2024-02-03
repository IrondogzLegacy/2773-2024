// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  XboxController joy = new XboxController(0);
  DriveCommand driveCommand = new DriveCommand(driveSubsystem, joy);
  CarDriveCommand carDriveCommand = new CarDriveCommand(driveSubsystem, joy);
  NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
  JoystickButton button = new JoystickButton(joy, 1);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(carDriveCommand);
    button.whileTrue(new RunCommand(() -> driveSubsystem.reset(), driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
