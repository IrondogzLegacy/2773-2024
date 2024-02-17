// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

/** Add your docs here. */
public class AutoMoveSubsystem {

    public AutoMoveSubsystem(NavigationSubsystem navigationSubsystem, DriveSubsystem driveSubsystem) {
        this.navSub = navigationSubsystem;
        this.driveSub = driveSubsystem;
    }

    NavigationSubsystem navSub;
    DriveSubsystem driveSub;

    public void rotateBotTo(double radians) {
        if ((navSub.angle + Math.PI) < radians) {
            while (navSub.angle > radians) {
                driveSub.rotate(-0.1);
            }
        } else {
            while (navSub.angle < radians) {
                driveSub.rotate(0.1);
            }
        }
    }

    //Currently not functioning
    public void rotateRobotTo(double radians) {
        while (!(navSub.angle > radians - 0.005 && navSub.angle < radians + 0.005)) {
            driveSub.rotate(0.1);
            navSub.angle = navSub.gyro.getAngle() / 180.0 * Math.PI;
        }
    }

    public void rotateWheelsTo(double radians) {
        while (!(navSub.fla > radians - 0.005 && navSub.fla < radians + 0.005)) {
            driveSub.directionalDrive(0, radians);
            SwerveModulePosition[] pos = driveSub.getPositions();
            navSub.fla = pos[0].angle.getRadians();
        }
    }

    public void movePolar(double radians, double distance) {
        rotateWheelsTo(radians);
        double y = Math.sin(radians) * distance;
        double x = Math.cos(radians) * distance;
        double cx = navSub.flx - Math.cos(radians) * distance;
        double cy = navSub.fly - Math.sin(radians) * distance;
        while (cy < y || cx < x) {
            driveSub.directionalDrive(1, radians);
        }
    }

    public void moveTo(double x, double y) {
        double cx = navSub.x;
        double cy = navSub.y;
        double dx = cx-x;
        double dy = cy-y;
        double distance = Math.sqrt(dx*dx + dy*dy);
        double radians = (Math.atan(dy/dx) * 180)/Math.PI;
        movePolar(radians, distance);
    }

    public void moveRelative(double x, double y) {
        double distance = Math.sqrt(x*x + y*y);
        double radians = (Math.atan(y/x) * 180)/Math.PI;
        movePolar(radians, distance);
    }
}
