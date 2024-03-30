// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ArmControlCommand extends Command {
  /** Creates a new ArmControlCommand. */
  private final XboxController armStick;
  private final ArmSubsystem armSubsystem;

  public ArmControlCommand(ArmSubsystem armSubsystem, XboxController armStick) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    this.armStick = armStick;
  }

  double holdAt;
  double endPosition;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean resetZero = armStick.getRawButton(7) && armStick.getRawButton(8);
    if (resetZero) {
      armSubsystem.resetArmEncoder();
    }
    if (armStick.getRawButton(7)) {
      armSubsystem.noSafety();
    }
    armSubsystem.rotateBy(
        -Constants.ControlArmSpeed * MathUtil.applyDeadband(armStick.getRightY(), Constants.ControllerDeadzone));
        // System.out.println("RightY: " + armStick.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
