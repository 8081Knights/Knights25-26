package org.firstinspires.ftc.teamcode.subsystems;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.HelperMethods.normalizeAngle;
import static java.lang.Math.*;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class Drive {

    private static void stopDrive(HardwareSoftware robot) {
        robot.FLdrive.setPower(0);
        robot.BLdrive.setPower(0);
        robot.FRdrive.setPower(0);
        robot.BRdrive.setPower(0);
    }

    /* ================= Target Pose ================= */
    public static class NewPositionOfRobot {
        public double newx, newy;
        public double newRotation;
        public double speed = 0.7;

        public NewPositionOfRobot(double x, double y, double rot) {
            newx = x;
            newy = y;
            newRotation = rot;
        }

        public NewPositionOfRobot(double x, double y, double rot, double spd) {
            newx = x;
            newy = y;
            newRotation = rot;
            speed = spd;
        }
    }

    /* ================= Current Pose ================= */
    public static class CurrentRobotPose {

        public double realRobotX, realRobotY, realRobotHeading;

        private double prevErrX = 0;
        private double prevErrY = 0;

        /* -------- Initialize (used once) -------- */
        public void init(HardwareSoftware hw, double x, double y, double heading) {
            realRobotX = x;
            realRobotY = y;
            realRobotHeading = heading;
        }

        /* -------- Update pose from OTOS -------- */
        public void updateRealRobotPositions(SparkFunOTOS.Pose2D pose) {
            realRobotX = pose.x;
            realRobotY = pose.y;
            realRobotHeading = normalizeAngle(pose.h);
        }

        /* ================= Motion Controller ================= */
        public double moveToSetPosition(NewPositionOfRobot target, HardwareSoftware robot) {

            /* ======== TUNING ======== */
            final double kP_xy = 0.08;
            final double kD_xy = 0.02;

            final double kP_turn = 0.7;

            final double deadbandXY = 0.5;               // inches
            final double deadbandH  = toRadians(2);      // radians

            final double minPowerXY = 0.05;
            /* ======================= */

            /* -------- Position error -------- */
            double errX = target.newx - realRobotX;
            double errY = target.newy - realRobotY;

            double distance = hypot(errX, errY);

            /* -------- Heading error -------- */
            double headingError = normalizeAngle(target.newRotation - realRobotHeading);

            /* -------- Derivative (damping) -------- */
            double dX = errX - prevErrX;
            double dY = errY - prevErrY;

            prevErrX = errX;
            prevErrY = errY;

            /* -------- XY PID -------- */
            double powX = (kP_xy * errX) - (kD_xy * dX);
            double powY = (kP_xy * errY) - (kD_xy * dY);

            /* -------- Deadbands -------- */
            if (abs(errX) < deadbandXY) powX = 0;
            if (abs(errY) < deadbandXY) powY = 0;

            /* -------- Min power -------- */
            if (powX != 0) powX = copySign(max(abs(powX), minPowerXY), powX);
            if (powY != 0) powY = copySign(max(abs(powY), minPowerXY), powY);

            powX = clamp(powX, -1, 1);
            powY = clamp(powY, -1, 1);

            /* -------- Rotation (gated) -------- */
            // Rotation authority increases as we get closer
            double turnScale = clamp(1.0 - (distance / 36.0), 0.4, 1.0);

            double rx = kP_turn * headingError * turnScale;

            // Phase 0: minimum rotation while moving
            double minTurn = 0.08;
            if (abs(rx) < minTurn && abs(headingError) > toRadians(3)) {
                rx = copySign(minTurn, headingError);
            }

            if (abs(headingError) < deadbandH) rx = 0;
            rx = clamp(rx, -0.5, 0.5);

            /* -------- Speed scaling -------- */
            double speedScale = clamp(distance / 12.0, 0.25, 1.0);
            double finalSpeed = target.speed * speedScale;

            /* -------- Field-centric transform -------- */
            double h = -realRobotHeading;
            double rotX = powX * cos(h) - powY * sin(h);
            double rotY = powX * sin(h) + powY * cos(h);

            /* -------- Mecanum -------- */
            double fl = -rotY - rotX + rx;
            double bl = -rotY + rotX + rx;
            double fr = -rotY + rotX - rx;
            double br = -rotY - rotX - rx;

            double max = max(1.0, max(
                    max(abs(fl), abs(fr)),
                    max(abs(bl), abs(br))
            ));

            robot.FLdrive.setPower(fl / max * finalSpeed);
            robot.BLdrive.setPower(bl / max * finalSpeed);
            robot.FRdrive.setPower(fr / max * finalSpeed);
            robot.BRdrive.setPower(br / max * finalSpeed);

            /* -------- Completion metric -------- */
            boolean atPos = distance < 0.7;
            boolean atHeading = abs(headingError) < toRadians(3);


            return distance;
        }

        /**
         * Rotates the robot in place to a target heading.
         *
         * @param targetHeading radians (field-centric)
         * @param robot hardware reference
         * @return true when rotation is complete
         */
        public boolean turnOnly(double targetHeading, HardwareSoftware robot) {

            // Compute shortest angular error
            double error = normalizeAngle(targetHeading - realRobotHeading);

            // Deadband (radians)
            double tolerance = Math.toRadians(2.0);
            if (Math.abs(error) < tolerance) {
                stopDrive(robot);
                return true;
            }

            // Proportional turn control
            double kP = 0.8;
            double rx = error * kP;

            // Clamp power
            rx = Math.max(-0.5, Math.min(0.5, rx));

            // Minimum power to overcome static friction
            double minTurn = 0.15;
            if (Math.abs(rx) < minTurn) {
                rx = Math.signum(rx) * minTurn;
            }

            // Apply pure rotation
            robot.FLdrive.setPower(rx);
            robot.BLdrive.setPower(rx);
            robot.FRdrive.setPower(-rx);
            robot.BRdrive.setPower(-rx);

            return false;
        }
    }
}