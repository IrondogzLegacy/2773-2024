// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class RotateArmToAngleCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private double angle;

  public RotateArmToAngleCommand(ArmSubsystem armSubsystem, double position) {
    this.armSubsystem = armSubsystem;
    this.angle = position;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.setAngle(angle); 
    System.out.println("Set arm to angle : " + angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished angle : " + armSubsystem.getRotationAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armSubsystem.atSetpoint();
  }
}
