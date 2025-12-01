package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.HelperMethods.normalizeAngle;
import static org.firstinspires.ftc.teamcode.subsystems.CameraSensor.initVision;

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

import org.firstinspires.ftc.teamcode.subsystems.Drive.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "TylerAuto")

public class TylerAuto extends LinearOpMode {
	//future competition auto,
	// need to work on getting the go to position to work,
	// but it should work with the go to position from driveRed

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


        robot.initGyro();


        visionPortal = initVision();

		//this is where you add all of the locations for the robot to go to


		robotPoses.add(new NewPositionOfRobot(0, -20, 0));
		robotPoses.add(new NewPositionOfRobot(20, -20, 0));
		robotPoses.add(new NewPositionOfRobot(20, 0, 0));
		robotPoses.add(new NewPositionOfRobot(10, 0, 0));
		robotPoses.add(new NewPositionOfRobot(10, 0, 0));
        robotPoses.add(new NewPositionOfRobot(10, 0, 0));
        robotPoses.add(new NewPositionOfRobot(10, 0, 0));
        robotPoses.add(new NewPositionOfRobot(10, 0, 0));
        robotPoses.add(new NewPositionOfRobot(10, 0, 0));




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

			cerror = currentPose.moveToSetPosition(robotPoses.get(currentInstruction), robot);
            cX = currentPose.cDx;
            cY = currentPose.cDy;
            cH = currentPose.cDh;

			telemetry.addData("cerror", cerror);

			telemetry.addData("cX", cX);
			telemetry.addData("cY", cY);
			telemetry.addData("cH", cH);


			//this is for the point scoring, not the wheels

			switch (currentInstruction) {
				case 0: {

					if (caseStopwatch.seconds() < 10) {
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
