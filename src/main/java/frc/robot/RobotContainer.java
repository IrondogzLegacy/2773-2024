// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Arm.ArmControlCommand;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Arm.ClimbCommand;
import frc.robot.Arm.ClimbSubsystem;
import frc.robot.Arm.LetGoCommand;
import frc.robot.Arm.RotateDownCommand;
import frc.robot.Arm.RotateUpCommand;
import frc.robot.Autonomous.AutoSubsystem;
import frc.robot.IntakeShooter.ControlledShootCommand;
import frc.robot.IntakeShooter.IntakeCommand;
import frc.robot.IntakeShooter.IntakeSubsystem;
import frc.robot.IntakeShooter.ReverseIntakeCommand;
import frc.robot.IntakeShooter.ShootCommand;
import frc.robot.IntakeShooter.ShooterSubsystem;
import frc.robot.Navigation.NavigationSubsystem;
import frc.robot.Autonomous.MoveDistanceAngleCommand;
import frc.robot.Autonomous.MoveRelativeCommand;
import frc.robot.Autonomous.MoveToCommand;

public class RobotContainer {
  // Controllers
  XboxController driveStick = new XboxController(0);
  XboxController armStick = new XboxController(1);

  // Subsystems
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  NavigationSubsystem navigationSubsystem = new NavigationSubsystem(driveSubsystem::getPositions);
  ArmSubsystem armSubsystem = new ArmSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  // AutoSubsystem autoMoveSubsystem = new AutoSubsystem(navigationSubsystem, driveSubsystem, armSubsystem, driveStick);
  ClimbSubsystem climbSubsystem = new ClimbSubsystem(armStick);

  // Commands from files
  DriveCommand driveCommand = new DriveCommand(driveSubsystem, driveStick, armStick, navigationSubsystem);
  ArmControlCommand armControlCommand = new ArmControlCommand(armSubsystem, armStick);
  //CarDriveCommand carDriveCommand = new CarDriveCommand(driveSubsystem, driveStick);
  // SwitchCommand switchCommand = new SwitchCommand(driveSubsystem, carDriveCommand, driveCommand);
  IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  // IntakeCommand intakeCommand1sec = new IntakeCommand(intakeSubsystem);
  // IntakeCommand intakeCommand3sec = new IntakeCommand(intakeSubsystem);
  ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intakeSubsystem);
  // ReverseShooterCommand reverseShooterCommand = new ReverseShooterCommand(intakeSubsystem);
  ShootCommand shootCommand = new ShootCommand(shooterSubsystem);
  ControlledShootCommand controlledShootCommand = new ControlledShootCommand(shooterSubsystem, armStick);
  // MoveDistanceAngleCommand moveDistanceAngleCommand = new MoveDistanceAngleCommand(autoMoveSubsystem);
  // MoveRelativeCommand moveRelativeCommand = new MoveRelativeCommand(0, 0, autoMoveSubsystem);
  // MoveToCommand moveToCommand = new MoveToCommand(0, 0, autoMoveSubsystem);
  RotateDownCommand rotateDownCommand = new RotateDownCommand(armSubsystem);
  RotateUpCommand rotateUpCommand = new RotateUpCommand(armSubsystem);
  ClimbCommand climbCommand = new ClimbCommand(climbSubsystem);
  LetGoCommand letGoCommand = new LetGoCommand(climbSubsystem);

  //Buttons
  JoystickButton intakeButton = new JoystickButton(armStick, 2);
  JoystickButton shootButton = new JoystickButton(armStick, 6);
  // JoystickButton switchButton = new JoystickButton(driveStick, 3);
  // JoystickButton resetMotorsButton = new JoystickButton(driveStick, 4);
  JoystickButton resetOrientationButton = new JoystickButton(driveStick, 7);
  JoystickButton raiseArmButton = new JoystickButton(armStick, 4);
  JoystickButton lowerArmButton = new JoystickButton(armStick, 3);
  // JoystickButton testButton = new JoystickButton(driveStick, 7);
  JoystickButton reverseIntakeButton = new JoystickButton(armStick, 1);
  // JoystickButton reverseShooterButton = new JoystickButton(armStick, 5);
  JoystickButton climbButton = new JoystickButton(armStick, 5);
  JoystickButton letGoButton = new JoystickButton(armStick, 8);

  // //Composite Commands
  // ParallelRaceGroup intake3sec = new ParallelRaceGroup(new WaitCommand(3),intakeCommand3sec); //for three seconds we intake
  // ParallelRaceGroup intake1sec = new ParallelRaceGroup(new WaitCommand(1), intakeCommand1sec); //intake for 1 second
  // ParallelRaceGroup shootWithIntake = new ParallelRaceGroup(new WaitCommand(3), shootCommand); //run the shooter for 3 seconds
  // ParallelCommandGroup intakeThenShoot = new ParallelCommandGroup(new WaitCommand(2).andThen(intake1sec), shootWithIntake);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    driveSubsystem.setDefaultCommand(driveCommand);
    armSubsystem.setDefaultCommand(armControlCommand);
    shooterSubsystem.setDefaultCommand(controlledShootCommand);
    // resetMotorsButton.whileTrue(new RunCommand(() -> driveSubsystem.resetMotors(), driveSubsystem));
    resetOrientationButton.onTrue(new InstantCommand(navigationSubsystem::resetOrientation));
    // switchButton.onTrue(switchCommand);
    intakeButton.whileTrue(intakeCommand);
    shootButton.whileTrue(shootCommand);
    raiseArmButton.whileTrue(rotateUpCommand);
    lowerArmButton.whileTrue(rotateDownCommand);
    // testButton.whileTrue(moveDistanceAngleCommand);
    // reverseShooterButton.whileTrue(reverseShooterCommand);
    reverseIntakeButton.whileTrue(reverseIntakeCommand);
    climbButton.whileTrue(climbCommand);
    letGoButton.whileTrue(letGoCommand);
  }

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // autonomous commands
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
