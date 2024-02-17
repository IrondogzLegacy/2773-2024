// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Arm.RotateDownCommand;
import frc.robot.Arm.RotateUpCommand;
import frc.robot.IntakeShooter.IntakeCommand;
import frc.robot.IntakeShooter.IntakeSubsystem;
import frc.robot.IntakeShooter.ReverseIntakeCommand;
import frc.robot.IntakeShooter.ReverseShooterCommand;
import frc.robot.IntakeShooter.ShootCommand;

public class RobotContainer {
  //Subsystems
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ArmSubsystem armSubsystem = new ArmSubsystem();
  
  //Controllers
  XboxController driveStick = new XboxController(0);
  XboxController armStick = new XboxController(1);

  
  //Commands from files
  IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand1sec = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand3sec = new IntakeCommand(intakeSubsystem);
  ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intakeSubsystem);
  ReverseShooterCommand reverseShooterCommand = new ReverseShooterCommand(intakeSubsystem);
  ShootCommand shootCommand = new ShootCommand(intakeSubsystem);
  ShootCommand shootWithIntakeCommand = new ShootCommand(intakeSubsystem);
  RotateDownCommand rotateDownCommand = new RotateDownCommand(armSubsystem);
  RotateUpCommand rotateUpCommand = new RotateUpCommand(armSubsystem);

  
  //Buttons
    JoystickButton intakeButton = new JoystickButton(driveStick, 1);
    JoystickButton shootButton = new JoystickButton(driveStick, 3);
    JoystickButton raiseArmButton = new JoystickButton(armStick, 4);
    JoystickButton lowerArmButton = new JoystickButton(armStick, 3);
    JoystickButton reverseIntakeButton = new JoystickButton(armStick, 1);
    JoystickButton reverseShooterButton = new JoystickButton(armStick, 2);


  //Instant Commands
  

  //Commands
  
  //Composite Commands
  ParallelRaceGroup intake3sec = new ParallelRaceGroup(new WaitCommand(3),intakeCommand3sec); //for three seconds we intake  
  ParallelRaceGroup intake1sec = new ParallelRaceGroup(new WaitCommand(1), intakeCommand1sec); //intake for 1 second
  ParallelRaceGroup shootWithIntake = new ParallelRaceGroup(new WaitCommand(3), shootWithIntakeCommand); //run the shooter for 3 seconds
  ParallelCommandGroup intakeThenShoot = new ParallelCommandGroup(new WaitCommand(2).andThen(intake1sec), shootWithIntake);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //Controller 0
    intakeButton.whileTrue(intakeCommand);
    shootButton.onTrue(shootCommand);
    
    //Controller 1
    reverseShooterButton.whileTrue(reverseShooterCommand);
    reverseIntakeButton.whileTrue(reverseIntakeCommand);
    raiseArmButton.whileTrue(rotateUpCommand);
    lowerArmButton.whileTrue(rotateDownCommand);
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
