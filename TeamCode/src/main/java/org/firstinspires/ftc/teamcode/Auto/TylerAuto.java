package org.firstinspires.ftc.teamcode.Auto;

import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TylerAuto")

public class TylerAuto extends LinearOpMode {

    HardwareSoftware robot = new HardwareSoftware();

    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    List<AprilTagDetection> detections = new ArrayList<>();
    int TAGID;
    final int frameWidth = 1280;
    int cameraBuffer = frameWidth / 3;

    static final double COUNTS_PER_INCH = 33.3;

    private ElapsedTime runtime = new ElapsedTime();

    VisionPortal.Builder vBuilder = new VisionPortal.Builder();

    List<Integer> obeliskTags = List.of(21, 22, 23);

    double[] initPositions = {0, 0, 0};

    List<NewPositionOfRobot> robotPoses = new ArrayList<>();

    int currentInstruction = 0;

    boolean isOkToMoveOn = false;

    ElapsedTime caseStopwatch = new ElapsedTime();

    CurrentRobotPose currentPose = new CurrentRobotPose();

    SparkFunOTOS.Pose2D pos;

    Long time = null;

    double cError;
    double cX;
    double cY;
    double cH;

    double cTreshold = .5;


    public void initThis() {

        robot.init(hardwareMap);

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
        robot.gyro.resetTracking();


        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();


        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTag)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .enableLiveView(true)
                .build();

        //this is where you add all of the locations for the robot to go to


        robotPoses.add(new NewPositionOfRobot(0, -5, 0));
        robotPoses.add(new NewPositionOfRobot(5, -5, 0));
        robotPoses.add(new NewPositionOfRobot(5, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));
        robotPoses.add(new NewPositionOfRobot(0, 0, 0));


//        robotPoses.add(new NewPositionOfRobot(0, 5, Math.PI / 2.0));
//        robotPoses.add(new NewPositionOfRobot(5, 5, Math.PI));
//        robotPoses.add(new NewPositionOfRobot(5, 0, Math.PI * (3 / 2.0)));
//        robotPoses.add(new NewPositionOfRobot(0, 0, 0));

        /* //starting pos: next to the big tower where you shoot the balls
        // see obelisk and shoot
        robotPoses.add(new NewPositionOfRobot(30, 0, 0));
        robotPoses.add(new NewPositionOfRobot(30, 0, -Math.PI*.5));

        // go down, grab first pattern
        robotPoses.add(new NewPositionOfRobot(30, 30, Math.PI*.75));
        robotPoses.add(new NewPositionOfRobot(10, 30, Math.PI*.75));

        // shoot pattern
        robotPoses.add(new NewPositionOfRobot(30, 0, -Math.PI*.5));

        // go down, grab 2nd pattern
        robotPoses.add(new NewPositionOfRobot(30, 54, Math.PI*.75));
        robotPoses.add(new NewPositionOfRobot(10, 54, Math.PI*.75));

        // shoot pattern
        robotPoses.add(new NewPositionOfRobot(30, 0, -Math.PI*.5));

        // go down, grab 3rd pattern
        robotPoses.add(new NewPositionOfRobot(30, 78, Math.PI*.75));
        robotPoses.add(new NewPositionOfRobot(10, 78, Math.PI*.75));

        // shoot pattern
        robotPoses.add(new NewPositionOfRobot(30, 0, -Math.PI*.5));

        */

        //maybe make it so that based on the pattern it will go to a different spot and get the balls so it does not need to sort it
        //TODO: add a detect obelisk method
        //TODO: add a sort method
        //TODO: add a shoot ball method
        //TODO: add a shoot using pattern method

        currentPose.init(robot, initPositions[0], initPositions[1], initPositions[2]);
    }

    public void runOpMode() throws InterruptedException {

        initThis();

        waitForStart();


        double cerror;


        while (opModeIsActive() && !isStopRequested()) {
            pos = robot.gyro.getPosition();
            telemetry.addData("Posx", pos.x);
            telemetry.addData("Posy", pos.y);
            telemetry.addData("Posh", pos.h);
            telemetry.update();

            currentPose.gyX = pos.x;
            currentPose.gyY = pos.y;
            currentPose.gyR = pos.h;

            currentPose.updateRealRobotPositions(pos);

            cerror = currentPose.moveToSetPosition(robotPoses.get(currentInstruction));

            telemetry.addData("cerror", cerror);

            telemetry.addData("cX", cX);
            telemetry.addData("cY", cY);
            telemetry.addData("cH", cH);


            //this is for the point scoring, not the wheels

            switch (currentInstruction) {
                case 0: {

                    if (caseStopwatch.seconds() < 2) {
                        isOkToMoveOn = false;
                    } else {
                        isOkToMoveOn = true;
                    }
                    //isOkToMoveOn = detectMotif();
                    break;
                }

                case 3:
                case 6:
                case 9: {
                    //sortBall();
                }

                case 1:
                case 4:
                case 7:
                case 10: {
                    if (caseStopwatch.seconds() < 4) {
                        isOkToMoveOn = false;
                    } else {
                        isOkToMoveOn = true;
                    }
                    //isOkToMoveOn = detectMotif();

                    break;
                }
            }


            //can add && isOkayToMoveOn
            if (Math.abs(cerror) < cTreshold || isOkToMoveOn) {
                caseStopwatch.reset();
                caseStopwatch.startTime();
                currentInstruction++;
            }
            telemetry.addData("cu", currentInstruction);

        }


    }


    /**
     * Represents the new position and rotation of the robot.
     */
    public class NewPositionOfRobot {
        boolean justDrive;
        double newx, newy;
        double newRotation;
        double speed = .4;

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
            realRobotHeading = initZ + normalizeAngle(-gyR);
        }

        /**
         * Moves the robot to a set position and returns the current error.
         *
         * @param setPose The target position and rotation.
         * @return The current error between the robot's position and the target position.
         */
        double moveToSetPosition(NewPositionOfRobot setPose) {
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


            double realSetX = powX * Math.cos(-realRobotHeading) - powY * Math.sin(-realRobotHeading);
            double realSetY = powX * Math.sin(-realRobotHeading) + powY * Math.cos(-realRobotHeading);

            telemetry.addData("powx", powX);
            telemetry.addData("powy", powY);
            telemetry.addData("realSetX", realSetX);
            telemetry.addData("realSetY", realSetY);
            telemetry.addData("rx", rx);


            double denominator = Math.max(Math.abs(powY) + Math.abs(powX) + Math.abs(rx), 1);

            robot.FLdrive.setPower(((-realSetY - realSetX + rx) / denominator) * setPose.speed);
            robot.BLdrive.setPower(((-realSetY + realSetX + rx) / denominator) * setPose.speed);
            robot.FRdrive.setPower(((-realSetY - realSetX - rx) / denominator) * setPose.speed);
            robot.BRdrive.setPower(((-realSetY + realSetX - rx) / denominator) * setPose.speed);

            cX = Math.abs(powdX);
            cY = Math.abs(powdY);
            cH = Math.abs(rx);
            currentError = cX + cY + cH;

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

    public void shootPurple() {

    }

    public void shootGreen() {

    }

    public boolean detectMotif() {
        detections = aprilTag.getDetections();
        if (!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (obeliskTags.contains(tag.id)) {
                    TAGID = tag.id;
                    return true;
                }
            }
        }
        return false;
    }


    public void shootMotif(int id) {
        // shoot the balls in respective pattern
    }

    public void sortBall() {

    }


}
