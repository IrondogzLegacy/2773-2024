// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ArmControlCommand extends Command {
  /** Creates a new ArmControlCommand. */
  private final XboxController armStick;
  private final ArmSubsystem armSubsystem;

  private PIDController rotateAnglePID = new PIDController(0.02, 0, 0);

  public ArmControlCommand(ArmSubsystem armSubsystem, XboxController armStick) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
    this.armStick = armStick; 
  }

  double holdAt;
  double endPosition;

  private void resetSetpoints() {
    holdAt = armSubsystem.getRotationAngle();
    rotateAnglePID.setSetpoint(holdAt);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateAnglePID.setTolerance(5);
    resetSetpoints();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean resetZero = armStick.getRawButton(7) && armStick.getRawButton(8);
    if (resetZero) {
      armSubsystem.resetArmEncoder();
      resetSetpoints();}
      boolean overrideZero = armStick.getRawButton(7);
    double minHoldAngle = overrideZero ? Constants.armMaxRotationOverride : 0;
    double maxHoldAngle = Constants.armMaxAngle;
    double minDistance = overrideZero ? Constants.armMaxPositionOverride : 0;
    double maxDistance = Constants.armMaxPosition;
    holdAt += -0.8 * MathUtil.applyDeadband(armStick.getLeftY(), Constants.ControllerDeadzone);
    holdAt = MathUtil.clamp(holdAt, minHoldAngle, maxHoldAngle);
    rotateAnglePID.setSetpoint(holdAt);
    double speed = rotateAnglePID.calculate(armSubsystem.getRotationAngle());
    speed = MathUtil.clamp(speed, -Constants.ArmMaxRotationSpeed, Constants.ArmMaxRotationSpeed);
    armSubsystem.rotate(speed);

    endPosition += -0.3 * MathUtil.applyDeadband(armStick.getRightY(), Constants.ControllerDeadzone);
    endPosition = MathUtil.clamp(endPosition, minDistance, maxDistance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
