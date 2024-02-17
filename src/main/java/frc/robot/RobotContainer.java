// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.sound.sampled.ReverbType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.IntakeShooter.IntakeCommand;
import frc.robot.IntakeShooter.IntakeSubsystem;
import frc.robot.IntakeShooter.ReverseIntakeCommand;
import frc.robot.IntakeShooter.ReverseShooterCommand;
import frc.robot.IntakeShooter.ShootCommand;
import frc.robot.Navigation.MoveDistanceAngleCommand;
import frc.robot.Navigation.NavigationSubsystem;
import frc.robot.pathfinding.UDPSubSystem;

public class RobotContainer {
  //Subsystems
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  NavigationSubsystem navigationSubsystem = new NavigationSubsystem(driveSubsystem::getPositions);
  ArmSubsystem armSubsystem = new ArmSubsystem();
  AutoSubsystem autoMoveSubsystem = new AutoSubsystem(navigationSubsystem, driveSubsystem, armSubsystem);
  UDPSubSystem udpSubSystem = new UDPSubSystem();
  
  //Controllers
  XboxController driveStick = new XboxController(0);
  XboxController armStick = new XboxController(1);

  
  //Commands from files
  DriveCommand driveCommand = new DriveCommand(driveSubsystem, driveStick, navigationSubsystem);
  CarDriveCommand carDriveCommand = new CarDriveCommand(driveSubsystem, driveStick);
  SwitchCommand switchCommand = new SwitchCommand(driveSubsystem, carDriveCommand, driveCommand);
  IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand1sec = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand3sec = new IntakeCommand(intakeSubsystem);
  ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intakeSubsystem);
  ReverseShooterCommand reverseShooterCommand = new ReverseShooterCommand(intakeSubsystem);
  ShootCommand shootCommand = new ShootCommand(intakeSubsystem);
  MoveDistanceAngleCommand moveDistanceAngleCommand = new MoveDistanceAngleCommand(autoMoveSubsystem);
  
  //Buttons
    JoystickButton intakeButton = new JoystickButton(driveStick, 1);
    JoystickButton shootButton = new JoystickButton(driveStick, 2);
    JoystickButton switchButton = new JoystickButton(driveStick, 3);
    JoystickButton resetMotorsButton = new JoystickButton(driveStick, 4);
    JoystickButton raiseArmButton = new JoystickButton(driveStick, 5);
    JoystickButton lowerArmButton = new JoystickButton(driveStick, 6);
    JoystickButton testButton = new JoystickButton(driveStick, 7);
    JoystickButton reverseIntakeButton = new JoystickButton(armStick, 1);
    JoystickButton reverseShooterButton = new JoystickButton(armStick, 2);


  //Instant Commands
  InstantCommand raiseArmCommand = new InstantCommand(armSubsystem::rotateUp);
  InstantCommand lowerArmCommand = new InstantCommand(armSubsystem::rotateDown);

  //Commands
  
  //Composite Commands
  ParallelRaceGroup intake3sec = new ParallelRaceGroup(new WaitCommand(3),intakeCommand3sec); //for three seconds we intake  
  ParallelRaceGroup intake1sec = new ParallelRaceGroup(new WaitCommand(1), intakeCommand1sec); //intake for 1 second
  ParallelRaceGroup shootWithIntake = new ParallelRaceGroup(new WaitCommand(3), shootCommand); //run the shooter for 3 seconds
  ParallelCommandGroup intakeThenShoot = new ParallelCommandGroup(new WaitCommand(2).andThen(intake1sec), shootWithIntake);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //Controller 0
    driveSubsystem.setDefaultCommand(driveCommand);
    //resetMotorsButton.whileTrue(new RunCommand(() -> driveSubsystem.resetMotors(), driveSubsystem));
    //switchButton.onTrue(switchCommand);
    intakeButton.whileTrue(intakeCommand);
    shootButton.onTrue(intakeThenShoot);
    raiseArmButton.whileTrue(raiseArmCommand);
    lowerArmButton.whileTrue(lowerArmCommand);
    testButton.whileTrue(moveDistanceAngleCommand);
    //Controller 1
    reverseShooterButton.whileTrue(reverseShooterCommand);
    reverseIntakeButton.whileTrue(reverseIntakeCommand);
  }

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

//autonomous commands
  public Command getRedMiddleAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
  public Command getBlueMiddleAutonomousCommand() { 
    return Commands.print("No autonomous command configured");
  }
  public Command getRedLeftAutoCommand() {
    return Commands.print("No autonomous command configured");
  }
  public Command getBlueLeftAutoCommand() {
    return Commands.print("No autonomous command configured");
  }
  public Command getRedRightAutoCommand() {
    return Commands.print("No autonomous command configured");
  }
  public Command getBlueRightAutoCommand() {
    return Commands.print("No autonomous command configured");
  }
}
