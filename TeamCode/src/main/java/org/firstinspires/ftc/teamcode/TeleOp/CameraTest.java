package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "cameraTest")
public class CameraTest extends OpMode {
	// start of camera auto tracking code
	//will try to turn and go to the april tag


	//AprilTagProcessor tagProcessor;
	VisionPortal visionPortal;
	AprilTagProcessor aprilTag;
	List<AprilTagDetection> detections;
	final int TAGID = 20;
	final int frameWidth = 1280;
	int cameraBuffer = frameWidth / 3;

	//private DcMotor         LDrive   = null;
	//private DcMotor         RDrive  = null;
	static final double COUNTS_PER_INCH = 33.3;

	private ElapsedTime runtime = new ElapsedTime();


	String initData = "";


	VisionPortal.Builder vBuilder = new VisionPortal.Builder();

	HardwareSoftware robot = new HardwareSoftware();


	//CRServo cameraServo;
	double range = 0;

	public void init() {

		robot.init(hardwareMap);

		robot.gyro.calibrateImu();
		robot.gyro.resetTracking();

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
		// Create the AprilTag processor
		//tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
		aprilTag = new AprilTagProcessor.Builder()
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setDrawTagOutline(true)
				.build();

		// Attach to the VisionPortal (this manages the camera stream)
		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
				.addProcessor(aprilTag)
				.build();

		//cameraServo = hardwareMap.get(CRServo.class, "servoExample");

		telemetry.addLine("Initialized. Press Play.");
		detections = aprilTag.getDetections();
		initData += aprilTag.getDetections().size();
		for (AprilTagDetection det : detections) {
			initData += "ID: " + det.id + " X: " + det.ftcPose.x;

			telemetry.addData("ID", det.id);
			telemetry.addData("Center", "(%.2f, %.2f)", det.center.x, det.center.y);
			telemetry.addData("Pose X", det.ftcPose.x);
			telemetry.addData("Pose Y", det.ftcPose.y);
			telemetry.addData("Heading (deg)", det.ftcPose.yaw);
			telemetry.addData("Range", det.ftcPose.range);
			range = det.ftcPose.range;
			telemetry.update();
		}

		//LDrive  = hardwareMap.get(DcMotor.class, "Ldrive");
		//RDrive = hardwareMap.get(DcMotor.class, "Rdrive");


		telemetry.update();

	}

	@Override
	public void loop() {
		telemetry.addData("initData", initData);

		detections = aprilTag.getDetections();
		for (AprilTagDetection det : detections) {
			telemetry.addData("ID", det.id);
			telemetry.addData("Center", "(%.2f, %.2f)", det.center.x, det.center.y);
			telemetry.addData("Pose X", det.ftcPose.x);
			telemetry.addData("Pose Y", det.ftcPose.y);
			telemetry.addData("Heading (deg)", det.ftcPose.yaw);
			telemetry.addData("Range", det.ftcPose.range);
			range = det.ftcPose.range;

			// cameraServo.setPower(.2);
//                    if (det.ftcPose.yaw < 0){
//                        cameraServo.setPower(tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.25);
//                    }
//                    if (det.ftcPose.yaw > 0){
//                        cameraServo.setPower(-.2);
//                    }
			//List<AprilTagDetection> result = tagProcessor.getDetections();
		}


		if (!detections.isEmpty()) {
			for (AprilTagDetection tag : detections) {
				//if (tag.id == TAGID) {
				telemetry.addData("TAG OUT", tag.center.x);
                                /*if (tag.center.x >= 0 && tag.center.x <= cameraBuffer) {
                                    cameraServo.setPower(-.5);
                                }
                                else if (tag.center.x >= (frameWidth - cameraBuffer) && tag.center.x <= frameWidth) {
                                    cameraServo.setPower(.5);
                                }
                                else {
                                    cameraServo.setPower(0);
                                }*/
				double num = -((tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.2);
				//cameraServo.setPower(num);
				telemetry.addData("thingy", (num));
				//}
				goToTag();
			}
		} else {
			telemetry.addData("TAG OUT", "NONE");
			//cameraServo.setPower(0);
		}

		telemetry.update();

	}

	public void continuousEncoderDrive(double speed,
									   double leftInches, double rightInches,
									   double timeoutS) {
		int newLeftBackTarget;
		int newRightBackTarget;
		int newRightFrontTarget;
		int newLeftFrontTarget;


		// Ensure that the OpMode is still active

		// Determine new target position, and pass to motor controller
		newLeftBackTarget = robot.FLdrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
		newRightBackTarget = robot.BLdrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
		newLeftFrontTarget = robot.BLdrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
		newRightFrontTarget = robot.BLdrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
		robot.BLdrive.setTargetPosition(newLeftBackTarget);
		robot.BRdrive.setTargetPosition(newRightBackTarget);
		robot.FLdrive.setTargetPosition(newLeftFrontTarget);
		robot.FRdrive.setTargetPosition(newRightFrontTarget);


		// Turn On RUN_TO_POSITION
		robot.FLdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.FRdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.BRdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		robot.BLdrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


		// reset the timeout time and start motion.
		runtime.reset();
		robot.FLdrive.setPower(Math.abs(speed));
		robot.FRdrive.setPower(Math.abs(speed));
		robot.BLdrive.setPower(Math.abs(speed));
		robot.BRdrive.setPower(Math.abs(speed));


		// keep looping while we are still active, and there is time left, and both motors are running.
		// Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
		// its target position, the motion will stop.  This is "safer" in the event that the robot will
		// always end the motion as soon as possible.
		// However, if you require that BOTH motors have finished their moves before the robot continues
		// onto the next step, use (isBusy() || isBusy()) in the loop test.
		while ((runtime.seconds() < timeoutS) &&
				(robot.FLdrive.isBusy() && robot.FRdrive.isBusy() && robot.BLdrive.isBusy() && robot.BRdrive.isBusy())) {

			// Display it for the driver.
			telemetry.addData("Running to", " %7d :%7d :%7d :%7d", newLeftBackTarget, newRightBackTarget);
			telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d",
					robot.FLdrive.getCurrentPosition(), robot.FRdrive.getCurrentPosition(), robot.BRdrive.getCurrentPosition(), robot.BLdrive.getCurrentPosition());
			telemetry.update();
		}
/*
            // Stop all motion;
            LDrive.setPower(0);
            RDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
            */


	}

	public void goToTag() {
		while (range > 10) {
			continuousEncoderDrive(0.6, 5, 5, 6);
			telemetry.update();
		}

	}
}

