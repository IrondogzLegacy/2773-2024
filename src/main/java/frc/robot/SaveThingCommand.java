// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Arm.ArmSubsystem;

public class SaveThingCommand extends Command {
  /** Creates a new SaveThingCommand. */
  int value;

  ArmSubsystem armSub;
  public SaveThingCommand(int value, ArmSubsystem armSubsystem) {
    addRequirements(armSubsystem);
    this.armSub = armSubsystem;
    this.value = value;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (value == 1) {
    armSub.middleSavedAngle = armSub.armRotationEncoder.getPosition();
    } else {
      armSub.sideSavedAngle = armSub.armRotationEncoder.getPosition();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
