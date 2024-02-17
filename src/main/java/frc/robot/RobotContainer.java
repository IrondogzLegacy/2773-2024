// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.IntakeShooter.IntakeCommand;
import frc.robot.IntakeShooter.IntakeSubsystem;
import frc.robot.IntakeShooter.ShootCommand;

public class RobotContainer {
  //Subsystems
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  
  //Controllers
  XboxController driveStick = new XboxController(0);
  
  //Commands from files
  IntakeCommand intakeCommand3sec = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand1sec = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);

  ShootCommand shootCommand = new ShootCommand(intakeSubsystem);
  ShootCommand shootCommand1 = new ShootCommand(intakeSubsystem);

  
  //Buttons
  JoystickButton intakeButton = new JoystickButton(driveStick, 1);
  JoystickButton shootButton = new JoystickButton(driveStick, 3);

  //Instant Commands
  

  //Composite Commands
  ParallelRaceGroup intake3sec = new ParallelRaceGroup(new WaitCommand(3),intakeCommand3sec); //for three seconds we intake  
  ParallelRaceGroup intake1sec = new ParallelRaceGroup(new WaitCommand(1), intakeCommand1sec); //intake for 1 second
  ParallelRaceGroup shootWithIntake = new ParallelRaceGroup(new WaitCommand(3), shootCommand); //run the shooter for 3 seconds
  ParallelCommandGroup intakeThenShoot = new ParallelCommandGroup(new WaitCommand(2).andThen(intake1sec), shootWithIntake);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    intakeButton.whileTrue(intakeCommand);
    shootButton.whileTrue(shootCommand1);
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
