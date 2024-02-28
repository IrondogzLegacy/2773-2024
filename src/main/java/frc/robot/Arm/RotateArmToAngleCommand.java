// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import java.util.function.Supplier;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class RotateArmToAngleCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private PIDController rotateAnglePID = new PIDController(0.02, 0, 0);
  private Supplier<Double> endAngle;
  private boolean dontStop;

  public RotateArmToAngleCommand(ArmSubsystem armSubsystem, double endAngle) {
    this(armSubsystem, () -> endAngle, false);
  }

  public RotateArmToAngleCommand(ArmSubsystem armSubsystem, double endAngle,
      boolean dontStop) {
    this(armSubsystem, () -> endAngle, dontStop);
  }

  
  public RotateArmToAngleCommand(ArmSubsystem armSubsystem, Supplier<Double> endAngle, boolean dontStop) {
    this.armSubsystem = armSubsystem;
    this.endAngle = endAngle;
    this.dontStop = dontStop;
    addRequirements(armSubsystem);
  }


  public static RotateArmToAngleCommand buildAngleMover(ArmSubsystem armSubsystem, double endAngle) {
    return new RotateArmToAngleCommand(armSubsystem, () -> endAngle, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateAnglePID.setSetpoint(endAngle.get());
    rotateAnglePID.setTolerance(0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = rotateAnglePID.calculate(armSubsystem.getRotationAngle());
    speed = MathUtil.clamp(speed, -Constants.ArmMaxRotationSpeed, Constants.ArmMaxRotationSpeed);
    armSubsystem.rotateWithSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.rotateStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (dontStop) {
      return false;
    }
    return rotateAnglePID.atSetpoint();
  }
}
