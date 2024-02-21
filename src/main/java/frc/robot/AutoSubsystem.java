// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Arm.ArmSubsystem;
import frc.robot.Navigation.NavigationSubsystem;

/** Add your docs here. */
public class AutoSubsystem {

    public AutoSubsystem(NavigationSubsystem navigationSubsystem, DriveSubsystem driveSubsystem,
            ArmSubsystem armSubsystem) {
        this.navSub = navigationSubsystem;
        this.driveSub = driveSubsystem;
        this.armSub = armSubsystem;
    }

    NavigationSubsystem navSub;
    DriveSubsystem driveSub;
    ArmSubsystem armSub;

    // Rotates the chassis direction to given radians
    public void rotateRobotTo(double radians) {
        while (!(navSub.angle > radians - 0.005 && navSub.angle < radians + 0.005)) {
            driveSub.rotate(0.1);
            navSub.angle = navSub.gyro.getAngle() / 180.0 * Math.PI;
        }
    }

    // Rotates the wheels to given radians, w/o moving chassis
    public void rotateWheelsTo(double radians) {
        while (!(navSub.fla > radians - 0.005 && navSub.fla < radians + 0.005)) {
            driveSub.directionalDrive(0, radians);
            SwerveModulePosition[] pos = driveSub.getPositions();
            navSub.fla = pos[0].angle.getRadians();
        }
    }

    // Moves towards a given polar coordinate
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

    // Moves to a given x,y coordinate relative to the field
    public void moveTo(double x, double y) {
        double cx = navSub.x;
        double cy = navSub.y;
        double dx = cx - x;
        double dy = cy - y;
        double distance = Math.sqrt(dx * dx + dy * dy);
        double radians = Math.atan(dy / dx);
        if (dx < 0 && dy > 0 && radians < 0) {
            radians += Math.PI;
        } else if (dx < 0 && dy < 0 && radians > 0) {
            radians -= Math.PI;
        }
        movePolar(radians, distance);
    }

    // Moves to a given x,y coordinate relative to the robots current position
    public void moveRelative(double x, double y) {
        double distance = Math.sqrt(x * x + y * y);
        double radians = Math.atan(y / x);
        /*
         * if (dx < 0 && dy > 0 && radians < 0) {
         * radians += Math.PI;
         * } else if(dx < 0 && dy < 0 && radians > 0) {
         * radians -= Math.PI;
         * }
         * movePolar(radians, distance);
         */
    }

    // Rotates the arm to the given radians
    public void setArmTo(double radians) {
        double currentAngle = armSub.getRotationAngle();
        double low = radians - 0.005;
        double high = radians + 0.005;
        // while (!(currentAngle > low && currentAngle < high)) {
        // while (currentAngle < low) {
        // armSub.rotate(0.1);
        // }
        // while (currentAngle > high) {
        // armSub.rotate(-0.1);
        // }
        // currentAngle = armSub.getRotationAngle();
        // }
    }
}