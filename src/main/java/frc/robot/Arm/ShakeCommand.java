// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ShakeCommand extends Command {
  private ArmSubsystem armSubsystem;

  /** Creates a new ShakeCommand. */
  public ShakeCommand(ArmSubsystem armSubsystem) {
    addRequirements(armSubsystem);
    this.armSubsystem = armSubsystem;
  }

  double angle;
  int state;
  Timer wait1 = new Timer();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angle = armSubsystem.angle();
    state = 0;
    armSubsystem.setAngle(0.1);
    wait1.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (state) {
      case 0:
        if (wait1.hasElapsed(0.5)) {
          wait1.stop();
          wait1.reset();
          wait1.start();
          state = 1;
          armSubsystem.setAngle(angle);
        }
        break;
      case 1:
        if (wait1.hasElapsed(0.5)) {
          wait1.stop();
          wait1.reset();
          state = 2;
        }
      default:
        
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == 2;
  }
}
