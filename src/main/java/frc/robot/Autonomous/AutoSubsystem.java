// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DriveSubsystem;
import frc.robot.SwerveDriveModule;
import frc.robot.Navigation.NavigationSubsystem;
import frc.robot.Constants;

public class AutoSubsystem extends SubsystemBase {
    public final NavigationSubsystem navSub;
    public final DriveSubsystem driveSub;
    private final XboxController joy;


    public AutoSubsystem(NavigationSubsystem navSub, DriveSubsystem driveSub, XboxController joy) {
        this.navSub = navSub;
        this.driveSub = driveSub;
        this.joy = joy;
    }

    // Rotates the chassis direction to given radians
    public void rotateRobotTo(double radians) {
        PIDController rotatePID = new PIDController(0.63, 0, 0);

        // rotatePID.setSetpoint(0);
        // double pos = -navSub.angle - radians;
        // while (pos < -Math.PI)
        //   pos += 2 * Math.PI;
        // while (pos >= Math.PI)
        //   pos -= 2 * Math.PI;
        // // -PI =< pos < PI

        // double direction = 1.0;
        // if (pos < -Math.PI / 2) {
        //   direction = -1.0;
        //   pos += Math.PI;
        // }
        // if (pos > Math.PI / 2) {
        //   direction = -1.0;
        //   pos -= Math.PI;
        // }

        rotatePID.setSetpoint(radians);

        double speedOfRotation = rotatePID.calculate(navSub.angle);   //navSub.angle is the angle of the robot
        speedOfRotation = MathUtil.clamp(speedOfRotation, -0.7, 0.7);
        rotateWheel(driveSub.flMotor, Math.PI/4);
        rotateWheel(driveSub.flMotor, 3 *Math.PI/4);
        rotateWheel(driveSub.flMotor, -3 * Math.PI/4);
        rotateWheel(driveSub.flMotor, -Math.PI/4);
        driveSub.flMotor.driveMotor.set(speedOfRotation);
        driveSub.frMotor.driveMotor.set(speedOfRotation);
        driveSub.blMotor.driveMotor.set(speedOfRotation);
        driveSub.brMotor.driveMotor.set(speedOfRotation);

        rotatePID.close();

        // driveMotor.set(0.1 * direction);

        // while (!(navSub.angle > radians - 0.005 && navSub.angle < radians + 0.005) && !joy.getRawButton(2)) {
        //     driveSub.rotate(0.1);
        //     navSub.angle = navSub.gyro.getAngle() / 180.0 * Math.PI;
        // }
    }

    public void rotateWheel(SwerveDriveModule motor, double radians) {
        PIDController pidRotate = new PIDController(0.63, 0, 0);

        pidRotate.setSetpoint(0);
        double pos = -navSub.angle - radians;
        while (pos < -Math.PI)
          pos += 2 * Math.PI;
        while (pos >= Math.PI)
          pos -= 2 * Math.PI;
        // -PI =< pos < PI

        if (pos < -Math.PI / 2) {
          pos += Math.PI;
        }
        if (pos > Math.PI / 2) {
          pos -= Math.PI;
        }
        double speedOfRotation = pidRotate.calculate(pos);
        speedOfRotation = MathUtil.clamp(speedOfRotation, -0.7, 0.7);

        motor.rotateMotor.set(speedOfRotation);
    }

    // Rotates the wheels to given radians, w/o moving chassis
    public void rotateWheelsTo(double radians) {
        while (!(navSub.fla > radians - 0.005 && navSub.fla < radians + 0.005) && !joy.getRawButton(2)) {
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
        double cx = navSub.x - Math.cos(radians) * distance;
        double cy = navSub.y - Math.sin(radians) * distance;
        while (cy < y || cx < x && !joy.getRawButton(2)) {
            driveSub.directionalDrive(0.1, radians);
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
        
        if (x < 0 && y > 0 && radians < 0) {
            radians += Math.PI;
        } else if(x < 0 && y < 0 && radians > 0) {
            radians -= Math.PI;
        }
        movePolar(radians, distance);
    }

}