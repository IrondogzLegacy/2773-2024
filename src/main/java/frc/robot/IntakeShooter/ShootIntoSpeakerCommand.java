// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ShootIntoSpeakerCommand extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public ShootIntoSpeakerCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem, shooterSubsystem);
  }

  public void reverseIntake1sec()
  {
    intakeSubsystem.reverseIntake();
    new WaitCommand(1);
    intakeSubsystem.stopIntake();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.startShooter();
    reverseIntake1sec();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.startIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intakeSubsystem.hasRing()) {return false;}
    return true;
  }
}
