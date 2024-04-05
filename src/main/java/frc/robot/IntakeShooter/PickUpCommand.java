// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakeShooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.OdometrySubsystem;

public class PickUpCommand extends Command {
  private IntakeSubsystem intakeSubsystem;
  private OdometrySubsystem odomSub;
  /** Creates a new IntakeCommand. */
  public PickUpCommand(IntakeSubsystem intakeSubsystem, OdometrySubsystem odometrySubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.odomSub = odometrySubsystem;
    // Use addRequirements() here to dec  lare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.startIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    odomSub.addSetPoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeSubsystem.hasRing();
  }
}
