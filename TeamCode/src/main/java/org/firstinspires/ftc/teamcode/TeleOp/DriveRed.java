package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Auto.TylerAuto;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name="DriveRed")
public class DriveRed extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();

    double[] initPositions = {0,0,0};

    DriveRed.CurrentRobotPose currentPose = new CurrentRobotPose();

    SparkFunOTOS.Pose2D pos;

    double cTreshold = .5;

    boolean isMovingToSetPos = false;

    boolean isOkToMoveOn = false;



    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.gyro().calibrateImu();
        robot.gyro().resetTracking();

        robot.gyro.setLinearUnit(DistanceUnit.INCH);
        robot.gyro.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.gyro.setOffset(offset);
        robot.gyro.setLinearScalar(1.0);
        robot.gyro.setAngularScalar(1.0);
        robot.gyro.calibrateImu();
        robot.gyro.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.gyro.setPosition(currentPosition);

        //this is where you add all of the locations for the robot to go to


        currentPose.init(robot,initPositions[0],initPositions[1],initPositions[2]);

    }

    double DS = 1;

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        SparkFunOTOS.Pose2D pos = robot.gyro().getPosition();

        double botHeading = -Math.toRadians(pos.h) + Math.PI;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        telemetry.addData("newX", rotX);
        telemetry.addData("newY", rotY);
        telemetry.addData("rx", rx);
        rotX = rotX * 1.1;  // Counteract imperfect strafing


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        if(!isMovingToSetPos){
            robot.FLdrive.setPower(((rotY + rotX + rx) / denominator) * DS);
            robot.BLdrive.setPower(((rotY - rotX + rx) / denominator) * DS);
            robot.FRdrive.setPower(((rotY - rotX - rx) / denominator) * DS);
            robot.BRdrive.setPower(((rotY + rotX - rx) / denominator) * DS);
        }


        if(gamepad1.a){
            driveToShootingPosFar(new NewPositionOfRobot(5,6,4));
        }

//        if (gamepad1.b) {
//            hw.Lucket().setPosition(.02);
//            hw.Rucket().setPosition(.09);
//        }
//        else if (gamepad1.a) {
//            hw.Lucket().setPosition(.64);
//            hw.Rucket().setPosition(.72);
//        }
//
//        if (gamepad1.dpad_up) {
//            hw.Linear().setTargetPosition(3000);
//            hw.Rinear().setTargetPosition(3000);
//            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Linear().setPower(1);
//            hw.Rinear().setPower(1);
//        }
//        if (gamepad1.dpad_left) {
//            hw.Linear().setTargetPosition(1500);
//            hw.Rinear().setTargetPosition(1500);
//            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Linear().setPower(1);
//            hw.Rinear().setPower(1);
//        }
//        if (gamepad1.dpad_down) {
//            hw.Linear().setTargetPosition(25);
//            hw.Rinear().setTargetPosition(25);
//            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Linear().setPower(1);
//            hw.Rinear().setPower(1);
//        }
//        if (gamepad1.right_bumper){
//            hw.InLinear().setPower(1);
//        }
//        else if (gamepad1.left_bumper){
//            hw.InLinear().setPower(-1);
//        }
//        else{
//            hw.InLinear().setPower(0);
//        }
//
//        if (gamepad1.right_trigger > .1) {
//            hw.Intake().setPower(gamepad1.right_trigger);
//        }
//        if (gamepad1.left_trigger > .1) {
//            hw.Intake().setPower(-gamepad1.left_trigger);
//        }
        if (gamepad1.share){
            robot.gyro.resetTracking();
        }



    }

    public void driveToShootingPosFar(NewPositionOfRobot setPos){
        isMovingToSetPos = true;

        double cerror;

        while (!isOkToMoveOn) {
            pos = robot.gyro.getPosition();
            telemetry.addData("Posx", pos.x);
            telemetry.addData("Posy", pos.y);
            telemetry.addData("Posh", pos.h);
            telemetry.update();

            currentPose.gyX = pos.x;
            currentPose.gyY = pos.y;
            currentPose.gyR = pos.h;

            currentPose.updateRealRobotPositions(pos);

            cerror = currentPose.moveToSetPosition(setPos);

            telemetry.addData("cerror", cerror);


            if (Math.abs(cerror) < cTreshold) {
                isOkToMoveOn = true;
            }

        }

        isMovingToSetPos = false;
        isOkToMoveOn = false;
    }


    /**
     * Represents the new position and rotation of the robot.
     */
    public class NewPositionOfRobot {
        boolean justDrive;
        double newx, newy;
        double newRotation;
        double speed = .8;
        /**
         * Sets the robot's future position and rotation.
         *
         * @param nx     The new x-coordinate.
         * @param ny     The new y-coordinate.
         * @param newRot The new rotation angle.
         */
        NewPositionOfRobot(double nx, double ny, double newRot) {
            this.newx = nx;
            this.newy = ny;
            this.newRotation = newRot;
            justDrive = true;
        }
        NewPositionOfRobot(double nx, double ny, double newRot, double setspeed) {
            this.newx = nx;
            this.newy = ny;
            this.newRotation = newRot;
            this.speed = setspeed;
            justDrive = true;
        }

    }

    public class CurrentRobotPose {
        HardwareSoftware robotHardwaremap;
        double realRobotX, realRobotY, realRobotHeading;
        double gyX, gyY, gyR;

        double initX, initY, initZ;

        /**
         * Initializes the robot's pose.
         *
         * @param hwMap The hardware map.
         * @param inX   The initial x-coordinate.
         * @param inY   The initial y-coordinate.
         * @param inz   The initial z-coordinate.
         */
        public void init(HardwareSoftware hwMap, double inX, double inY, double inz) {
            this.robotHardwaremap = hwMap;
            this.initX = inX;
            this.initY = inY;
            this.initZ = inz;
        }

        /**
         * Updates the robot's real positions based on gyroscope values.
         *
         * @param gyroValue The current gyroscope position.
         */
        public void updateRealRobotPositions(SparkFunOTOS.Pose2D gyroValue) {
            gyX = gyroValue.x;
            gyY = gyroValue.y;
            gyR = gyroValue.h;

            realRobotX = initX + gyX;
            realRobotY = initY + gyY;
            realRobotHeading = initZ + normalizeAngle(gyR);
        }

        /**
         * Moves the robot to a set position and returns the current error.
         *
         * @param setPose The target position and rotation.
         * @return The current error between the robot's position and the target position.
         */
        double moveToSetPosition(DriveRed.NewPositionOfRobot setPose) {
            double currentError = 0;
            double powY, powX, rx = 0;
            double powdY, powdX;

            powdX = setPose.newx - realRobotX;
            powdY = setPose.newy - realRobotY;

            if (powdX > 3 || powdX < -3) {
                powX = Math.signum(powdX);
            } else {
                powX = powdX/3;
            }

            if (powdY > 3 || powdY < -3) {
                powY = Math.signum(powdY);
            } else {
                powY = powdY/3;
            }

            double[] altAngles = new double[3];
            double[] diffAngles = new double[3];

            altAngles[0] =  setPose.newRotation - 2*Math.PI;
            altAngles[1] =  setPose.newRotation            ;
            altAngles[2] =  setPose.newRotation + 2*Math.PI;

            for (int i = 0; i < 3; ++i) {
                diffAngles[i] = altAngles[i] - realRobotHeading;
            }

            Arrays.sort(diffAngles);

            int goodindex = 0;

            if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[0])) {
                goodindex =1;
            }
            if (Math.abs(diffAngles[2]) < Math.abs(diffAngles[0])) {
                goodindex = 2;
            }
            if (Math.abs(diffAngles[2]) < Math.abs(diffAngles[1])) {
                goodindex = 2;
            }
            if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[2])) {
                goodindex = 1;
            }

            rx = diffAngles[goodindex];

            telemetry.addData("diffIn0", diffAngles[0]);
            telemetry.addData("diffIn1", diffAngles[1]);
            telemetry.addData("diffIn2", diffAngles[2]);


            double realSetY = powY * Math.cos(realRobotHeading) - powX * Math.sin(realRobotHeading);
            double realSetX = powY * Math.sin(realRobotHeading) + powX * Math.cos(realRobotHeading);

            telemetry.addData("powx", powX);
            telemetry.addData("powy", powY);
            telemetry.addData("realSetX", realSetX);
            telemetry.addData("realSetY", realSetY);
            telemetry.addData("rx", rx);




            double denominator = Math.max(Math.abs(powY) + Math.abs(powX) + Math.abs(rx), 1);

            robot.FLdrive.setPower((( -realSetY - realSetX - rx) / denominator) * setPose.speed);
            robot.BLdrive.setPower((( -realSetY + realSetX - rx) / denominator) * setPose.speed);
            robot.FRdrive.setPower((( -realSetY + realSetX + rx) / denominator) * setPose.speed);
            robot.BRdrive.setPower((( -realSetY - realSetX + rx) / denominator) * setPose.speed);
            currentError = Math.abs(powdX) + Math.abs(powdY) + Math.abs(rx);

            return currentError;
        }

    }

    public static double normalizeAngle(double angle) {
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }
}
