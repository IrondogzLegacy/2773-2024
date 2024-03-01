// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.IntakeShooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ControlledShootCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
    XboxController joy;

    public ControlledShootCommand(ShooterSubsystem shooterSubsystem, XboxController joy) {
      this.shooterSubsystem = shooterSubsystem;
      this.joy = joy;
      addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.runShooter(joy.getRightTriggerAxis() * Constants.shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}