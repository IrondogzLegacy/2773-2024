// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveSubsystem;
import frc.robot.Navigation.NavigationSubsystem;

public class MoveToCommand extends Command {

  AutoSubsystem autoSub;
  NavigationSubsystem navSub;
  DriveSubsystem driveSub;
  double x;
  double y;
  double gy;
  double gx;
  double dx;
  double dy;
  double distance = Math.sqrt(dx * dx + dy * dy);
  double radians = Math.atan(dy / dx);
  double ix;
  double iy;
  double dfx;
  double dfy;

  double tolerance = 0.1;

  /** Creates a new MoveToCommand. */
  public MoveToCommand(double x, double y, AutoSubsystem autoSubsystem) {
    addRequirements(autoSubsystem);
    this.autoSub = autoSubsystem;
    this.gx = x;
    this.gy = y;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navSub = autoSub.navSub;
    driveSub = autoSub.driveSub;
    ix = x;
    iy = y;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = navSub.x;
    y = navSub.y;
    dx = -(x - gx);
    dy = -(y - gy);
    distance = Math.sqrt(dx * dx + dy * dy);
    radians = Math.atan(dy / dx);
    if (dx < 0 && dy > 0 && radians < 0) {
        radians += Math.PI;
    } else if (dx < 0 && dy < 0 && radians > 0) {
        radians -= Math.PI;
    }

    // gy = Math.sin(radians) * distance;
    // gx = Math.cos(radians) * distance;
    // cx = navSub.x - Math.cos(radians) * distance;
    // cy = navSub.y - Math.sin(radians) * distance;
    // while (cy < gy || cx < gx && !joy.getRawButton(2)) {
    //     driveSub.directionalDrive(0.1, radians);
    // }

    // if (!(x > gx - tolerance && x < gx + tolerance) || !(x > gx - tolerance && x < gx + tolerance)) {
    //   driveSub.directionalDrive(0.1, radians);
    // }

    dfx = Math.abs(x) - Math.abs(ix);
    dfy = Math.abs(y) - Math.abs(iy);
    if (!(dfx > dx) && !(dfy > dy)) {
      driveSub.directionalDrive(0.1, radians);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!(dfx > dx) && !(dfy > dy));
  }
}
