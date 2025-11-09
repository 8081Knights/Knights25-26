package org.firstinspires.ftc.teamcode.TeleOp;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.io.InvalidClassException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp(name = "DriveRed")
public class DriveRed extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();

    double[] initPositions = {0, 0, 0};

    DriveRed.CurrentRobotPose currentPose = new CurrentRobotPose();

    DecimalFormat df = new DecimalFormat("#.##");


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

    double farShootingPos = 0.45;

    double closeShootingPos = 0.5;

    double sorterServoPos = 0.5;


    boolean showTelem = false;

    double diffGyroStartingPosX = 0;
    double diffGyroStartingPosY = 0;

    int redTagId = 24;
    int blueTagId = 20;

    // assuming field size roughly 144 x 144 inches (FTC field)
    Pose2D redTagPos = new Pose2D(DistanceUnit.INCH, 132, 120, AngleUnit.DEGREES, 225);
    Pose2D blueTagPos = new Pose2D(DistanceUnit.INCH, 12, 120, AngleUnit.DEGREES, 315);

    Pose2D absPosOfGryoStart = null;

    Pose2D absPosOfRobot = null;


    //telemetry, not used for calculations
    Pose2D relativePosToTarget = null;

    boolean knowsAbsRobotPos = false;

    boolean hasMotif = false;
    //the position of the green ball is the ones digit of the motif id
    int[] motifTagIds = {21, 22, 23};

    int greenBallPos;

    AprilTagDetection det = null;

    double cError;
    double cX;
    double cY;
    double cH;


    //this is the red teleop code
    // it can detect balls by color
    // use april tags
    // know where it is and where its gyro-origin is
    // go to an absolute position on the field

    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.gyro.calibrateImu();
        robot.gyro.resetTracking();

        robot.gyro.setLinearUnit(DistanceUnit.INCH);
        robot.gyro.setAngularUnit(AngleUnit.RADIANS);
        //old robot
        //SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        //new robot
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
                .addProcessor(aprilTag)
                .addProcessor(colorLocatorGreen)
                .addProcessor(colorLocatorPurple)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .build();


        // Start the live camera stream to the Driver Station preview window


        //this is where you add all of the locations for the robot to go to


        currentPose.init(robot, initPositions[0], initPositions[1], initPositions[2]);


    }

    //TODO: find out if the april tag math works,
    // make sure that the position of the robot is actually changing how i think it is
    // hello

    @Override
    public void loop() {
        telemetry.clear();
        //current blob telemetry is commented out
        handleBlobs();
        detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            telemetry.addLine("HAS TAG");
            for (AprilTagDetection detec : detections) {
                det = detec;
                if (absPosOfGryoStart == null) {
                    det = detec;
                    if (det.id == redTagId) {
                        findAbsRobotPos(redTagPos);
                        calculateGyroStartPos();
                    }
                    if (det.id == blueTagId) {
                        findAbsRobotPos(blueTagPos);
                        calculateGyroStartPos();
                    }
                }
                if (!hasMotif) {
                    if (det.id == motifTagIds[0]) {
                        greenBallPos = 0;
                        hasMotif = true;
                    }
                    if (det.id == motifTagIds[1]) {
                        greenBallPos = 1;
                        hasMotif = true;
                    }
                    if (det.id == motifTagIds[2]) {
                        greenBallPos = 2;
                        hasMotif = true;
                    }
                }
                if (gamepad1.right_bumper) {
                    showTelem = true;
                }
                if (gamepad1.left_bumper) {
                    showTelem = false;
                }
            }
        }


        if (!isMovingToSetPos) {
            manualHeadlessDrive();
        } else {
            updateAutoDrive();
        }

        if (gamepad1.x) {
            stopAutoMove();
        }

        telemetry.addData("Gyro X: ", robot.gyro.getPosition().x);
        telemetry.addData("Gyro Y: ", robot.gyro.getPosition().y);
        telemetry.addData("Gyro H: ", robot.gyro.getPosition().h);
        telemetry.addData("knows gyro starting pos", absPosOfGryoStart != null);
        if (absPosOfGryoStart != null) {
            telemetry.addData("startingPosGyro", pose2DtoString(absPosOfGryoStart));
            updateAbsoluteRobotPos();

        }
        if (absPosOfRobot != null) {
            telemetry.addData("currentRobotPos", pose2DtoString(absPosOfRobot));
        }

        /*
        if (det != null) {
            telemetry.addData("ID", det.id);
            telemetry.addData("Center", "(%.2f, %.2f)", det.center.x, det.center.y);
            if (det.ftcPose == null) {
                telemetry.addData("IDK WHY", "IDK WHY");
            } else {
                telemetry.addData("Pose X", det.ftcPose.x);
                telemetry.addData("Pose Y", det.ftcPose.y);
                telemetry.addData("Heading (deg)", det.ftcPose.yaw);
                telemetry.addData("Range", det.ftcPose.range);
                double calulatedX = det.ftcPose.range * Math.cos(det.ftcPose.yaw);
                telemetry.addData("calulated X", calulatedX);
                telemetry.addData("diffX", calulatedX - det.ftcPose.x);
            }
            telemetry.update();
        }
        */

        //TODO: figure out where the launch zone is
        //either this should be based on the april tags, or just make sure that
        // the gyro is reset in the same spot every time
        if (gamepad1.a) {
            //Pose2D pos = getRelativeDiff(new Pose2D(DistanceUnit.INCH, 40, 40, AngleUnit.RADIANS, 0));
            //relativePosToTarget = pos;
            //NewPositionOfRobot pose = new NewPositionOfRobot(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));
            startAutoMove(new NewPositionOfRobot(0, 40, 0));
        }

        if (gamepad1.b) {
            robot.gyro.resetTracking();
            absPosOfRobot = null;
            absPosOfGryoStart = null;
            det = null;
        }

        if (gamepad1.y) {
            SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();
            currentPose.updateRealRobotPositions(pos);
            String formattedX = String.format("%.2f", currentPose.realRobotX);
            String formattedY = String.format("%.2f", currentPose.realRobotY);
            telemetry.addLine("Current Position  X: " + formattedX + "Y: " + formattedY);
        }

        if (gamepad2.a) {
            robot.flyWheel.setPower(-0.7);
        } else if (gamepad2.x) {
            robot.flyWheel.setPower(-0.85);
        } else {
            robot.flyWheel.setPower(0);
        }

        if (gamepad2.b) {
            telemetry.addData("b is pressed", "");
            robot.flyWheelRotator1.setPosition(farShootingPos + 0.02);
            robot.flyWheelRotator2.setPosition(farShootingPos + 0.02);
        }
        if (gamepad2.y) {
            telemetry.addData("y is pressed", "");
            robot.flyWheelRotator1.setPosition(farShootingPos + 0.07);
            robot.flyWheelRotator2.setPosition(farShootingPos + 0.07);
        }


        telemetry.addData("flyWheelPos1:", robot.flyWheelRotator1.getPosition());
        telemetry.addData("flyWheelPos2:", robot.flyWheelRotator2.getPosition());

        telemetry.addData("cError", cError);
        telemetry.addData("cX", cX);
        telemetry.addData("cY", cY);
        telemetry.addData("cH", cH);


        if (gamepad2.right_trigger > 0.5) {
            robot.intake.setPower(0.95);
            //uh I think this works 🥀
        } else if (gamepad2.left_trigger > 0.5) {
            robot.intake.setPower(-0.95);
        } else {
            robot.intake.setPower(0);
        }

        //added methods to find the positions that work, once these are used then you can do set position ones
        // eventually, have two positions that are trapping a ball on the right and the left
        //can just toggle between these two positions

        if (gamepad2.dpad_right) {
            robot.sorterServo.setPosition(0.4);
            telemetry.addData("sorterServoPos", robot.sorterServo.getPosition());
        }

        if (gamepad2.dpad_left) {
            robot.sorterServo.setPosition(0.8);
            telemetry.addData("sorterServoPos", robot.sorterServo.getPosition());
        }

        telemetry.update();


    }

    /**
     * if you want to use color blobs,
     * the fields from the currentBlob and circle fit,
     * also the distance from blob variable too
     */
    public void handleBlobs() {
        updateBlobs();

        //telemetry.addLine("Circularity Radius Center");

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

    public void updateBlobs() {
        blobs = colorLocatorPurple.getBlobs();

        blobs.addAll(colorLocatorGreen.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50, 20000, blobs);  // filter out very small blobs.

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blobs);

        if (!blobs.isEmpty()) {
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
        double botHeading = -pos.h;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        robot.FLdrive.setPower(((rotY + rotX + rx) / denominator));
        robot.BLdrive.setPower(((rotY - rotX + rx) / denominator));
        robot.FRdrive.setPower(((rotY - rotX - rx) / denominator));
        robot.BRdrive.setPower(((rotY + rotX - rx) / denominator));
    }

    public void manualMechanumDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        //current thing
//        robot.FLdrive.setPower(y - rx - x);
//        robot.FRdrive.setPower(y + rx + x);
//        robot.BLdrive.setPower(y - rx + x);
//        robot.BRdrive.setPower(y + rx - x);

//        robot.FLdrive.setPower(y + x + rx);
//        robot.BLdrive.setPower(y - x + rx);
//        robot.FRdrive.setPower(y - x - rx);
//        robot.BRdrive.setPower(y + x - rx);

        //headless
        robot.FLdrive.setPower(y + rx - x);
        robot.FRdrive.setPower(y - rx - x);
        robot.BLdrive.setPower(y + rx + x);
        robot.BRdrive.setPower(y - rx + x);
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
        relativePosToTarget = null;

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
     * this should be for the first time when you detect an april tag and just have that position
     *
     * @param absTagPose the absolute positon of the april tag on the field
     */
    public void findAbsRobotPos(Pose2D absTagPose) {
        boolean foundImportantTag = false;
        double tagHeadingRad = Math.toRadians(absTagPose.getHeading(AngleUnit.DEGREES));

        //need the red tag and blue tag id on the target
        if (!detections.isEmpty()) {
            for (AprilTagDetection det : detections) {
                //skips code if unwanted tag
                if (det.id == 21 || det.id == 22 || det.id == 23 || foundImportantTag) {
                    continue;
                }

                if (det.id == blueTagId) {
                    foundImportantTag = true;
                }

                if (det.id == redTagId) {
                    foundImportantTag = true;
                }

                // rotate detection offsets (relative to tag) into field frame
                double relX = det.ftcPose.x;
                double relY = det.ftcPose.y;

                double rotatedX = relX * Math.cos(tagHeadingRad) - relY * Math.sin(tagHeadingRad);
                double rotatedY = relX * Math.sin(tagHeadingRad) + relY * Math.cos(tagHeadingRad);

                //these are the current position of the robot relative to the field, not the gyro
                double fieldX = absTagPose.getX(DistanceUnit.INCH) + rotatedX;
                double fieldY = absTagPose.getY(DistanceUnit.INCH) - rotatedY;

                double fieldHeading = AngleUnit.normalizeDegrees(
                        absTagPose.getHeading(AngleUnit.DEGREES) + det.ftcPose.yaw
                );

                absPosOfRobot = new Pose2D(DistanceUnit.INCH, fieldX, fieldY, AngleUnit.DEGREES, fieldHeading);
            }

        }

    }

    /**
     * this calculates where the gyro-origin is based on the absolute current position of the robot
     */
    public void calculateGyroStartPos() {
        if (absPosOfRobot != null) {
            double absGyroX = absPosOfRobot.getX(DistanceUnit.INCH) - robot.gyro.getPosition().x;
            double absGyroY = absPosOfRobot.getY(DistanceUnit.INCH) - robot.gyro.getPosition().y;
            absPosOfGryoStart = new Pose2D(DistanceUnit.INCH, absGyroX, absGyroY, AngleUnit.RADIANS, 0);
        }
    }

    /**
     * this should be used to update the current field centric robot position based on knowing where the gyro origin is
     */
    public void updateAbsoluteRobotPos() {
        if (absPosOfGryoStart != null) {
            double tempX = absPosOfGryoStart.getX(DistanceUnit.INCH) + robot.gyro.getPosition().x;
            double tempY = absPosOfGryoStart.getY(DistanceUnit.INCH) + robot.gyro.getPosition().y;
            double tempRot = absPosOfGryoStart.getHeading(AngleUnit.RADIANS) + robot.gyro.getPosition().h;

            absPosOfRobot = new Pose2D(DistanceUnit.INCH, tempX, tempY, AngleUnit.RADIANS, tempRot);
        }
    }

    /**
     * calculated difference from absolute position of target and absolute position of gyro-origin
     *
     * @param absPosOfTarget target position
     * @return position relative to the gyro-origin
     */
    public Pose2D getRelativeDiff(Pose2D absPosOfTarget) {
        Pose2D diff = null;
        if (absPosOfGryoStart != null) {
            double x = absPosOfTarget.getX(DistanceUnit.INCH) - absPosOfGryoStart.getX(DistanceUnit.INCH);
            double y = absPosOfTarget.getY(DistanceUnit.INCH) - absPosOfGryoStart.getY(DistanceUnit.INCH);
            double h = absPosOfTarget.getHeading(AngleUnit.RADIANS) - absPosOfGryoStart.getHeading(AngleUnit.RADIANS);

            diff = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, h);
        }

        return diff;
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
                powX = powdX / 3;
            }

            if (powdY > 3 || powdY < -3) {
                powY = Math.signum(powdY);
            } else {
                powY = powdY / 3;
            }

            double[] altAngles = new double[3];
            double[] diffAngles = new double[3];

            altAngles[0] = setPose.newRotation - 2 * Math.PI;
            altAngles[1] = setPose.newRotation;
            altAngles[2] = setPose.newRotation + 2 * Math.PI;

            for (int i = 0; i < 3; ++i) {
                diffAngles[i] = altAngles[i] - realRobotHeading;
            }

            Arrays.sort(diffAngles);

            int goodindex = 0;

            if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[0])) {
                goodindex = 1;
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


            double dx = setPose.newx - realRobotX;
            double dy = setPose.newy - realRobotY;

            double botHeading = -realRobotHeading;
            double realSetX = dx * Math.cos(botHeading) - dy * Math.sin(botHeading);
            double realSetY = dx * Math.sin(botHeading) + dy * Math.cos(botHeading);

            telemetry.addData("powx", powX);
            telemetry.addData("powy", powY);
            telemetry.addData("realSetX", realSetX);
            telemetry.addData("realSetY", realSetY);
            telemetry.addData("rx", rx);

            //basically just does headless until it gets to the right position
            double denominator = Math.max(Math.abs(powY) + Math.abs(powX) + Math.abs(rx), 1);

            robot.FLdrive.setPower(((-realSetY - realSetX - rx) / denominator) * setPose.speed);
            robot.BLdrive.setPower(((-realSetY + realSetX - rx) / denominator) * setPose.speed);
            robot.FRdrive.setPower(((-realSetY + realSetX + rx) / denominator) * setPose.speed);
            robot.BRdrive.setPower(((-realSetY - realSetX + rx) / denominator) * setPose.speed);
            cX = Math.abs(powdX);
            cY = Math.abs(powdY);
            cH = Math.abs(rx);
            currentError = cX + cY + cH;
            cError = currentError;
            return currentError;
        }

    }

    /**
     * normalizes an angle - because it is cyclical, keeps it in range of (0, 2pi)
     */
    public static double normalizeAngle(double angle) {
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public String pose2DtoString(Pose2D pos) {
        StringBuilder s = new StringBuilder("(Pose2D) x=");
        s.append(df.format(pos.getX(DistanceUnit.INCH)));
        s.append(",\n y=");
        s.append(df.format(pos.getY(DistanceUnit.INCH)));
        s.append(",\n h=");
        s.append(df.format(pos.getHeading(AngleUnit.RADIANS)));
        return s.toString();
    }
}
