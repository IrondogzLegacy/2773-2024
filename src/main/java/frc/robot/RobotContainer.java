// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
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
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Arm.ArmControlCommand;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Arm.ClimbCommand;
import frc.robot.Arm.ClimbSubsystem;
import frc.robot.Arm.LetGoCommand;
import frc.robot.Arm.PDownCommand;
import frc.robot.Arm.PUpCommand;
import frc.robot.Arm.RotateArmToAngleCommand;
import frc.robot.Arm.RotateDownCommand;
import frc.robot.Arm.RotateUpCommand;
import frc.robot.Autonomous.DriveToWallCommand;
import frc.robot.Autonomous.MoveDirectionCommand;
import frc.robot.Autonomous.PolarMoveCommand;
import frc.robot.Autonomous.RotateRobotCommand;
import frc.robot.IntakeShooter.ControlledShootCommand;
import frc.robot.IntakeShooter.IntakeCommand;
import frc.robot.IntakeShooter.IntakeSubsystem;
import frc.robot.IntakeShooter.PickUpCommand;
import frc.robot.IntakeShooter.PickUpCommand;
import frc.robot.IntakeShooter.ReverseIntakeCommand;
import frc.robot.IntakeShooter.ReverseShooterCommand;
import frc.robot.IntakeShooter.ShootCommand;
import frc.robot.IntakeShooter.ShootIntoAmpCommand;
import frc.robot.IntakeShooter.ShootIntoSpeakerCommand;
import frc.robot.IntakeShooter.ShooterSubsystem;
import frc.robot.Navigation.NavigationSubsystem;
import frc.robot.Navigation.OdometrySubsystem;
import frc.robot.Navigation.TagSubsystem;
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
  ClimbSubsystem climbSubsystem = new ClimbSubsystem(armStick);
  OdometrySubsystem odometrySubsystem = new OdometrySubsystem(navigationSubsystem);
  TagSubsystem tagSubsystem = new TagSubsystem(odometrySubsystem);

  // Commands from files
    //Drive Commands
  DriveCommand driveCommand = new DriveCommand(driveSubsystem, driveStick, armStick, navigationSubsystem);
  //CarDriveCommand carDriveCommand = new CarDriveCommand(driveSubsystem, driveStick);
  // SwitchCommand switchCommand = new SwitchCommand(driveSubsystem, carDriveCommand, driveCommand);
  // MoveDistanceAngleCommand moveDistanceAngleCommand = new MoveDistanceAngleCommand(autoMoveSubsystem);
  // MoveRelativeCommand moveRelativeCommand = new MoveRelativeCommand(0, 0, autoMoveSubsystem);
  Command move1Right1UpCommand = new ParallelRaceGroup(
    new MoveRelativeCommand(1, 1, odometrySubsystem, driveSubsystem),
    new WaitCommand(2));
  Command move1m45deg = new ParallelRaceGroup(
    new PolarMoveCommand(-0.75 * Math.PI, 1, driveSubsystem, odometrySubsystem),
    new WaitCommand(2)
  ).andThen(new DriveCommand(driveSubsystem, driveStick, armStick, navigationSubsystem));
  Command moveAngleCommand = new ParallelRaceGroup(
    new MoveRelativeCommand(1, 0, odometrySubsystem, driveSubsystem),
    new WaitCommand(2)
    ).andThen(new ParallelRaceGroup(
    new MoveRelativeCommand(0, 1, odometrySubsystem, driveSubsystem),
    new WaitCommand(2)));

    //Arm Commands
  ArmControlCommand armControlCommand = new ArmControlCommand(armSubsystem, armStick);
  RotateDownCommand rotateDownCommand = new RotateDownCommand(armSubsystem);
  RotateUpCommand rotateUpCommand = new RotateUpCommand(armSubsystem);
  ClimbCommand climbCommand = new ClimbCommand(climbSubsystem);
  LetGoCommand letGoCommand = new LetGoCommand(climbSubsystem);
    //Intake / Shooter Commands
  IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand1sec = new IntakeCommand(intakeSubsystem);
  IntakeCommand intakeCommand3sec = new IntakeCommand(intakeSubsystem);
  ReverseIntakeCommand reverseIntakeCommand = new ReverseIntakeCommand(intakeSubsystem);
  ReverseShooterCommand reverseShooterCommand = new ReverseShooterCommand(shooterSubsystem);
  ShootCommand shootCommand = new ShootCommand(shooterSubsystem);
  ShootCommand shootCommandWithIntake = new ShootCommand(shooterSubsystem);
  ControlledShootCommand controlledShootCommand = new ControlledShootCommand(shooterSubsystem);
  PUpCommand PUpCommand = new PUpCommand(armSubsystem);
  PDownCommand PDownCommand = new PDownCommand(armSubsystem);
  //Autonomous Commands
  RotateArmToAngleCommand rotateToSpeaker = new RotateArmToAngleCommand(armSubsystem, 0.3);
  RotateArmToAngleCommand rotateToAmp = new RotateArmToAngleCommand(armSubsystem, 1.1);
  ShootIntoSpeakerCommand shootIntoSpeakerCommand = new ShootIntoSpeakerCommand(intakeSubsystem, shooterSubsystem);
  ShootIntoAmpCommand shootIntoAmpCommand = new ShootIntoAmpCommand(intakeSubsystem, shooterSubsystem);
  ParallelCommandGroup shootIntoAmpCommandComposite = new ParallelCommandGroup();
  
  

  //Buttons
    //driveStick
  // JoystickButton switchButton = new JoystickButton(driveStick, 3);
  // JoystickButton resetMotorsButton = new JoystickButton(driveStick, 4);
  JoystickButton resetOrientationButton = new JoystickButton(driveStick, 7);
    
  //armStick
    JoystickButton intakeButton = new JoystickButton(armStick, 2);
    JoystickButton shootButton = new JoystickButton(armStick, 6);

  JoystickButton sideSpeakerShootButton = new JoystickButton(armStick, 4);
  JoystickButton middleSpeakerShootButton = new JoystickButton(armStick, 3);
  JoystickButton reverseIntakeButton = new JoystickButton(armStick, 1);
  JoystickButton reverseShooterButton = new JoystickButton(armStick, 5);
    POVButton dpadDownButton = new POVButton(armStick, 0);
    POVButton dpadRightButton = new POVButton(armStick, 90);
    POVButton dpadLeftButton = new POVButton(armStick, -90);
    POVButton dpadUpButton = new POVButton(armStick, 180);
  
  //Instant Commands

  // //Composite Commands
  ParallelRaceGroup intake3sec = new ParallelRaceGroup(new WaitCommand(3),intakeCommand3sec); //for three seconds we intake
  ParallelRaceGroup intake1sec = new ParallelRaceGroup(new WaitCommand(1), intakeCommand1sec); //intake for 1 second
  ParallelRaceGroup shootWithIntake = new ParallelRaceGroup(new WaitCommand(3), shootCommandWithIntake); //run the shooter for 3 seconds
  ParallelCommandGroup intakeThenShoot = new ParallelCommandGroup(new WaitCommand(2).andThen(intake1sec), shootWithIntake);
  Command angleShootCommand(Double angle) {
    return new ParallelRaceGroup(
    new RotateArmToAngleCommand(armSubsystem, angle),
    new ShootCommand(shooterSubsystem),
    new WaitCommand(1.5)
  ).andThen(new ParallelRaceGroup(
    new IntakeCommand(intakeSubsystem),
    new ShootCommand(shooterSubsystem),
    new WaitCommand(1)
  )); }

  
  Command middleShootCommand() {
    return new ParallelRaceGroup(
    new RotateArmToAngleCommand(armSubsystem, Constants.middleShootAngle),
    new ShootCommand(shooterSubsystem),
    new WaitCommand(1.5) //Maybe less
  ).andThen(new ParallelRaceGroup(
    new IntakeCommand(intakeSubsystem),
    new ShootCommand(shooterSubsystem),
    new WaitCommand(1)
  )); }

  Command sideShootCommand() {return new ParallelRaceGroup(
    new RotateArmToAngleCommand(armSubsystem, Constants.sideShootAngle),
    new ShootCommand(shooterSubsystem),
    new WaitCommand(1.5)
  ).andThen(new ParallelRaceGroup(
    new IntakeCommand(intakeSubsystem),
    new ShootCommand(shooterSubsystem),
    new WaitCommand(1)
  ));}
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //Default Commands
      driveSubsystem.setDefaultCommand(driveCommand);
      armSubsystem.setDefaultCommand(armControlCommand);
    
    //DriveStick
      // resetMotorsButton.whileTrue(new RunCommand(() -> driveSubsystem.resetMotors(), driveSubsystem));
      resetOrientationButton.onTrue(new InstantCommand(navigationSubsystem::resetOrientation));
      // switchButton.onTrue(switchCommand);
      //button 2
      //button 3
      //button 4
      //dPad Buttons on DriveStick


    //ArmStick
      intakeButton.whileTrue(intakeCommand); //Button 2
      shootButton.whileTrue(shootCommand); //Button 6
      middleSpeakerShootButton.onTrue(middleShootCommand()); //Button 3
      sideSpeakerShootButton.onTrue(sideShootCommand()); //Button 3
      // reverseShooterButton.whileTrue(reverseShooterCommand); //Button 5
      // reverseShooterButton.onTrue(new ShakeCommand(armSubsystem));
      reverseIntakeButton.whileTrue(reverseIntakeCommand); //Button 1
      //dPad Buttons on ArmStick
      dpadDownButton.whileTrue(climbCommand); //down arrow
      dpadUpButton.whileTrue(letGoCommand); //up arrow
      dpadRightButton.onTrue(PUpCommand);
      dpadLeftButton.onTrue(PDownCommand);
    //Overrides
      //Arm button 7 --> arm override
      //Arm buttons 7 & 8 --> arm reset
  }

  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //commands for autonomous
  

  // autonomous commands
  public Command testCommand() {
    return new ParallelRaceGroup(
      new PolarMoveCommand(-0.75 * Math.PI, 1, driveSubsystem, odometrySubsystem),
      new WaitCommand(2)
    ).andThen(new StopCommand(driveSubsystem));
  }

  public Command middleInnerAutoCommand() {
    return timed(middleShootCommand(), 2).andThen(new ParallelCommandGroup(
        timed(new PolarMoveCommand(-1.0/2 * Math.PI, Constants.betweenMiddleStartAndInsideNote + Constants.extraIntakeNeeded, driveSubsystem, odometrySubsystem), 2),
        timed(new RotateArmToAngleCommand(armSubsystem, 0), 2),
        timed(new PickUpCommand(intakeSubsystem), 2)
    )).andThen(new ParallelCommandGroup(
      timed(new RotateArmToAngleCommand(armSubsystem, Constants.middleShootAngle), 1.5),
      timed(new ShootCommand(shooterSubsystem), 1.5),
      timed(new PolarMoveCommand(1.0/2 * Math.PI, Constants.betweenMiddleStartAndInsideNote + Constants.extraIntakeNeeded, driveSubsystem, odometrySubsystem), 2)
    )).andThen(new ParallelCommandGroup(
      timed(new IntakeCommand(intakeSubsystem), 1),
      timed(new ShootCommand(shooterSubsystem), 1)
    )).andThen(new ParallelCommandGroup(
        timed(new RotateArmToAngleCommand(armSubsystem, 0), 1.5),
        timed(new PolarMoveCommand(1.0/2 * Math.PI, Constants.betweenMiddleStartAndInsideNote, driveSubsystem, odometrySubsystem), 2)
    )).andThen(
        timed(new RotateRobotCommand(-1.0/2 * Math.PI, navigationSubsystem, driveSubsystem), 1)
    ).andThen(new ParallelCommandGroup(
        timed(new PolarMoveCommand(1.0/2 * Math.PI, Constants.betweenInsideNotes + Constants.extraIntakeNeeded, driveSubsystem, odometrySubsystem), 2),
        timed(new PickUpCommand(intakeSubsystem), 2)
    )).andThen(
      timed(new PolarMoveCommand(-1.0/2 * Math.PI, Constants.betweenInsideNotes + Constants.extraIntakeNeeded, driveSubsystem, odometrySubsystem), 2)
    ).andThen(
      timed(new RotateRobotCommand(1.0/2 * Math.PI, navigationSubsystem, driveSubsystem), 1)
    ).andThen(new ParallelCommandGroup(
      timed(new RotateArmToAngleCommand(armSubsystem, Constants.middleShootAngle), 1.5),
      timed(new ShootCommand(shooterSubsystem), 1.5),
      timed(new PolarMoveCommand(1.0/2 * Math.PI, Constants.betweenMiddleStartAndInsideNote, driveSubsystem, odometrySubsystem), 2)
    )).andThen(new ParallelCommandGroup(
      timed(new IntakeCommand(intakeSubsystem), 1),
      timed(new ShootCommand(shooterSubsystem), 1)
    )).andThen(new ParallelCommandGroup(
        timed(new RotateArmToAngleCommand(armSubsystem, 0), 1),
        timed(new PolarMoveCommand(1.0/2 * Math.PI, Constants.betweenMiddleStartAndInsideNote, driveSubsystem, odometrySubsystem), 2)
    )).andThen(
        timed(new RotateRobotCommand(1.0/2 * Math.PI, navigationSubsystem, driveSubsystem), 1)
    ).andThen(new ParallelCommandGroup(
        timed(new PolarMoveCommand(1.0/2 * Math.PI, Constants.betweenInsideNotes + Constants.extraIntakeNeeded, driveSubsystem, odometrySubsystem), 2),
        timed(new PickUpCommand(intakeSubsystem), 2.5)
    )).andThen(
      timed(new PolarMoveCommand(-1.0/2 * Math.PI, Constants.betweenInsideNotes + Constants.extraIntakeNeeded, driveSubsystem, odometrySubsystem), 2)
    ).andThen(
      timed(new RotateRobotCommand(-1.0/2 * Math.PI, navigationSubsystem, driveSubsystem), 1)
    ).andThen(new ParallelCommandGroup(
      timed(new RotateArmToAngleCommand(armSubsystem, Constants.middleShootAngle), 1),
      timed(new ShootCommand(shooterSubsystem), 1),
      timed(new PolarMoveCommand(1.0/2 * Math.PI, Constants.betweenMiddleStartAndInsideNote, driveSubsystem, odometrySubsystem), 2)
    )).andThen(new ParallelCommandGroup(
      timed(new IntakeCommand(intakeSubsystem), 1),
      timed(new ShootCommand(shooterSubsystem), 1)
    )).andThen(
      timed(new RotateArmToAngleCommand(armSubsystem, 0), 1)
    );
  }

  public Command oppositeAmpOuterAutoCommand() {
    return new ParallelRaceGroup(
      timed(new PolarMoveCommand(-1/2 * Math.PI, Constants.outsideAutoInit1, driveSubsystem, odometrySubsystem), 1.5),
      new RotateArmToAngleCommand(armSubsystem, Constants.sideShootAngle)
    ).andThen(
      timed(new RotateRobotCommand(30 * (Math.PI/180), navigationSubsystem, driveSubsystem), 1)
    ).andThen(new ParallelRaceGroup(
      timed(new DriveToWallCommand(), 1)
    ));
  }

  public Command angledMiddleAutoCommand() {
    return middleShootCommand().andThen(
      
    );
  }

  public Command leftAutoCommand() {
    return middleShootCommand().andThen(
      
    );
  }

  public Command rotate90Command() {
    return middleShootCommand();
  }

  public Command getRedLeftAutoCommand() {
    return new ParallelRaceGroup(
      new MoveRelativeCommand(1, 0, odometrySubsystem, driveSubsystem),
      new WaitCommand(2)
    ).andThen(new ParallelRaceGroup(
      new MoveRelativeCommand(0, 1, odometrySubsystem, driveSubsystem),
      new WaitCommand(2))
    ).andThen(new DriveCommand(driveSubsystem, driveStick, armStick, navigationSubsystem));
  }

  public Command middlePositionShootCommand() {
    return middleShootCommand().andThen(new StopCommand(driveSubsystem));
  }

  public Command sidePositionShootCommand() {
    return sideShootCommand().andThen(new StopCommand(driveSubsystem));
  }
  
  

  public Command getRedRightAutoCommand() {
    return new ParallelRaceGroup(
      new MoveDirectionCommand(-0.25 * Math.PI, driveSubsystem, odometrySubsystem),
      new WaitCommand(0.5)
    ).andThen(new StopCommand(driveSubsystem));
  }

  public Command getBlueRightAutoCommand() {
    return Commands.print("No autonomous command configured");
  }

  public Command timed(Command command, double time) {
    return new ParallelRaceGroup(command, new WaitCommand(time));
  }
}
