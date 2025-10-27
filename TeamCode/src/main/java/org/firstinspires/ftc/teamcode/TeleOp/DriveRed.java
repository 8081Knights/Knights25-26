package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

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

    boolean autoJustStopped = false;


    NewPositionOfRobot currentTarget = null;

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    ArrayList<AprilTagDetection> detections;

    Circle circleFit = null;

    ColorBlobLocatorProcessor.Blob currentBlob = null;

    List<ColorBlobLocatorProcessor.Blob> blobs = null;

    ColorBlobLocatorProcessor colorLocatorPurple = null;

    ColorBlobLocatorProcessor colorLocatorGreen = null;

    double distanceFromBlob;

    double farShootingPos = 0.5;

    double closeShootingPos = 0.5;

    double sorterServoPos = 0;

    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.gyro.calibrateImu();
        robot.gyro.resetTracking();

        robot.gyro.setLinearUnit(DistanceUnit.INCH);
        robot.gyro.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(6.186, 0.7, 0);
        robot.gyro.setOffset(offset);
        robot.gyro.setLinearScalar(1.0);
        robot.gyro.setAngularScalar(1.0);
        robot.gyro.calibrateImu();
        robot.gyro.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.gyro.setPosition(currentPosition);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(colorLocatorGreen)
                .addProcessor(colorLocatorPurple)
                .build();

        //this is where you add all of the locations for the robot to go to


        currentPose.init(robot,initPositions[0],initPositions[1],initPositions[2]);



    }

    @Override
    public void loop() {
        telemetry.clear();
        //current blob telemetry is commented out
        handleBlobs();
        detections = aprilTag.getDetections();
        if (!detections.isEmpty() && gamepad1.right_bumper) {
            for (AprilTagDetection det : detections) {
                telemetry.addData("ID", det.id);
                telemetry.addData("Center", "(%.2f, %.2f)", det.center.x, det.center.y);
                telemetry.addData("Pose X", det.ftcPose.x);
                telemetry.addData("Pose Y", det.ftcPose.y);
                telemetry.addData("Heading (deg)", det.ftcPose.yaw);
                telemetry.addData("Range", det.ftcPose.range);
                telemetry.update();
            }
        }


        if (!isMovingToSetPos) {
            manualMechanumDrive();
        } else {
            updateAutoDrive();
        }

        if (gamepad1.x) {
            stopAutoMove();
        }

        telemetry.addData("Gyro X: ", robot.gyro.getPosition().x);
        telemetry.addData("Gyro Y: ", robot.gyro.getPosition().y);
        telemetry.addData("Gyro H: ", robot.gyro.getPosition().h);


        //TODO: figure out where the launch zone is
        //either this should be based on the april tags, or just make sure that
        // the gyro is reset in the same spot every time
        if (gamepad1.a) {
            startAutoMove(new NewPositionOfRobot(0, 40, 0));
        }

        if (gamepad1.b) {
            robot.gyro.resetTracking();
        }

        if (gamepad1.y) {
            SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();
            currentPose.updateRealRobotPositions(pos);
            String formattedX = String.format("%.2f", currentPose.realRobotX);
            String formattedY = String.format("%.2f", currentPose.realRobotY);
            telemetry.addLine("Current Position  X: " + formattedX + "Y: " + formattedY);
        }

        if(gamepad2.a){
            robot.flyWheel.setPower(0.5);
        }
        if(gamepad2.b) {
            robot.flyWheelRotator1.setPosition(farShootingPos);
            robot.flyWheelRotator2.setPosition(farShootingPos);
        }
        if(gamepad2.y){
            robot.flyWheelRotator1.setPosition(closeShootingPos);
            robot.flyWheelRotator2.setPosition(closeShootingPos);
        if(gamepad2.x){
            robot.intake.setPower(0.5);
            //uh I think this works 🥀
        }
        //added methods to find the positions that work, once these are used then you can do set position ones
            // eventually, have two positions that are trapping a ball on the right and the left
            //can just toggle between these two positions
        if(gamepad2.right_bumper){
            sorterServoPos += 0.1;
        }
        if(gamepad2.left_bumper) {
            sorterServoPos -= 0.1;
        }

        if(gamepad2.dpad_up){
            robot.sorterServo.setPosition(sorterServoPos);
            telemetry.addData("sorterServoPos", sorterServoPos);
        }




        }

    }

    /**
     * if you want to use color blobs,
     * the fields from the currentBlob and circle fit,
     * also the distance from blob variable too
     */
    public void handleBlobs(){
        updateBlobs();

        telemetry.addLine("Circularity Radius Center");

        double ARTIFACT_REAL_WIDTH_CM = 12.7;
        double FOCAL_LENGTH_IN_PIXELS = 424.4;

        // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
        for (ColorBlobLocatorProcessor.Blob b : blobs) {
            currentBlob = b;
            circleFit = currentBlob.getCircle();
            //telemetry.addLine("HELLLLLLLLLLOOOOOOO " + String.format("%5.3f      %3d     (%3d,%3d)",
                    //currentBlob.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
            telemetry.update();


            if (!blobs.isEmpty()) {
                // Assuming you care about the largest blob
                ColorBlobLocatorProcessor.Blob largestBlob = blobs.get(0);

                // Get the width of the bounding box around the blob in pixels
                double pixelWidth = 2 * circleFit.getRadius();

                // Calculate the distance
                distanceFromBlob = 0.9 * (ARTIFACT_REAL_WIDTH_CM * FOCAL_LENGTH_IN_PIXELS) / pixelWidth;

                // the multiplier 0.9 is used to calibrate the distance. could use some fine tuning
                // Send the information to the Driver Station
                //telemetry.addData("Distance to Artifact", "%.2f cm", distanceFromBlob);

                //telemetry.addLine("HELLLLLLLLLLOOOOOOO " + String.format("%5.3f      %3d     (%3d,%3d)",
                 //       currentBlob.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
                //telemetry.addData("Path", "Complete");
                telemetry.update();
            }
        }

    }

    public void updateBlobs(){
        blobs = colorLocatorPurple.getBlobs();

        blobs.addAll(colorLocatorGreen.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
            50, 20000, blobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blobs);

        if(!blobs.isEmpty()) {
            currentBlob = blobs.get(0);
        }
    }

    /**
     * this method handles headless math and controls robot
     */
    public void manualHeadlessDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();
        double botHeading = -Math.toRadians(pos.h) + Math.PI;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        robot.FLdrive.setPower(((rotY + rotX + rx) / denominator));
        robot.BLdrive.setPower(((rotY - rotX + rx) / denominator));
        robot.FRdrive.setPower(((rotY - rotX - rx) / denominator));
        robot.BRdrive.setPower(((rotY + rotX - rx) / denominator));
    }

    public void manualMechanumDrive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        robot.FLdrive.setPower(y - rx - x);
        robot.FRdrive.setPower(y + rx + x);
        robot.BLdrive.setPower(y - rx + x);
        robot.BRdrive.setPower(y + rx - x);
    }


    public void startAutoMove(NewPositionOfRobot target) {
        isMovingToSetPos = true;
        currentTarget = target;
    }

    public void updateAutoDrive() {
        SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();
        currentPose.updateRealRobotPositions(pos);

        double error = currentPose.moveToSetPosition(currentTarget);

        telemetry.addData("AutoMoving", true);
        telemetry.addData("Error", error);

        if (Math.abs(error) < cTreshold) {
            stopAutoMove();
        }
    }

    public void stopAutoMove() {
        isMovingToSetPos = false;
        currentTarget = null;

        robot.FLdrive.setPower(0);
        robot.FRdrive.setPower(0);
        robot.BLdrive.setPower(0);
        robot.BRdrive.setPower(0);
        telemetry.clear();
        telemetry.addData("AutoMove: ", "Stopped");
        telemetry.update();
        autoJustStopped = true;
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
