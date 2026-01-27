package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.HelperMethods.*;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;

import android.graphics.Color;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import android.util.Size;


import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.subsystems.CameraSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drive.*;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "DriveRed")
public class DriveRed extends OpMode {
	//current teleop for matches, both colors
	//MATCHES
	//TODO: need to do auto for this comp
	// then after that get the hooded shooter and turntable to work
	// then make sure the other code(turn and go to ball, shooting autonomously, drive stuff)
	// are all standard and implemented properly
	HardwareSoftware robot = new HardwareSoftware();

	double[] initPositions = {0, 0, 0};

	CurrentRobotPose currentPose = new CurrentRobotPose();

	DecimalFormat df = new DecimalFormat("#.##");

	SparkFunOTOS.Pose2D pos;

	double cTreshold = .5;

	boolean isMovingToSetPos = false;

	boolean isOkToMoveOn = false;

	boolean autoJustStopped = false;


	NewPositionOfRobot currentTarget = null;


	ArrayList<AprilTagDetection> detections;

	Circle circleFit = null;

	ColorBlobLocatorProcessor.Blob currentBlob = null;

	List<ColorBlobLocatorProcessor.Blob> blobs = null;


	double distanceFromBlob;

	double farShootingPos = 0.7;

	double closeShootingPos = 0.5;

	double sorterServoPos = 0.5;


	boolean showTelem = true;

	double diffGyroStartingPosX = 0;
	double diffGyroStartingPosY = 0;

	int redTagId = 24;
	int blueTagId = 20;

	double hoodPosition = 0.5;

	// assuming field size roughly 144 x 144 inches (FTC field)
	Pose2D redTagPos = new Pose2D(DistanceUnit.INCH, 132, 120, AngleUnit.DEGREES, 225);
	Pose2D blueTagPos = new Pose2D(DistanceUnit.INCH, 12, 120, AngleUnit.DEGREES, 315);
	Pose2D currentTagPos = null;

	Pose2D absPosOfGryoStart = null;

	Pose2D absPosOfRobot = null;


	//telemetry, not used for calculations
	Pose2D relativePosToTarget = null;

	boolean knowsAbsRobotPos = false;

	boolean hasMotif = false;
	//the position of the green ball is the ones digit of the motif id
	int[] motifTagIds = {21, 22, 23};

	int greenBallPos;

	int targetFlyWheelVelo = 0;

	VisionPortal visionPortal;

	AprilTagDetection det = null;

	double cError;
	double cX;
	double cY;
	double cH;

	int numDetections = 0;
	double autoFlywheelVelo = 700;

	CameraSensor camera;

	private ColorBlobLocatorProcessor colorLocatorPurple = null;

	private ColorBlobLocatorProcessor colorLocatorGreen = null;

	double turntablePos = 0.5;

	private AprilTagProcessor aprilTag;
	static ArrayList<VisionProcessor> processors = new ArrayList<>();

	double prevRange = 0;

	private ElapsedTime rampTime = new ElapsedTime();

	boolean isRamping = false;

	double flyWheelVelo = 0;

	double maxFlyWheelVelo = 1400;

	private final double targetRampTime = 5;

	double s2high = 0.95;
	double s2low = 0.63;

	double s1high = 0.63;
	double s1low = 0.31;

	private ElapsedTime sort1Time = new ElapsedTime();
	private ElapsedTime sort2Time = new ElapsedTime();

	boolean servoAPressed = false;

	boolean servoBPressed = false;
	boolean prevLeftBumper = false;
	boolean prevRightBumper = false;
	private boolean rightVelo;


	double green = .5;
	double purple = .666;
	double red = .279;

	AprilTagPoseFtc poseFtc = null;

	int MAXFLYWHEEL = 1400;

	boolean isManualFlywheel = false;




	//this is the red teleop code
	// it can detect balls by color,
	// use april tags,
	// go to relative position on field based on gyro-origin
	// WIP: know where it is and where its gyro-origin is
	// WIP: go to an absolute position on the field


	@Override
	public void init() {

		robot.init(hardwareMap);

		robot.initGyro();

		sort1Time.reset();
		sort2Time.reset();

		//visionPortal = camera.initVision();
		aprilTag = new AprilTagProcessor.Builder()
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setDrawTagOutline(true)
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


		VisionPortal.Builder o = new VisionPortal.Builder();
		o.addProcessor(aprilTag)
				.addProcessor(colorLocatorGreen)
				.addProcessor(colorLocatorPurple)
				.setCameraResolution(new Size(640, 480))
				.setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
				.enableLiveView(false);

		visionPortal = o.build();
		// Start the live camera stream to the Driver Station preview window


		currentPose.init(robot, initPositions[0], initPositions[1], initPositions[2]);


	}

	@Override
	public void loop() {
		telemetry.clear();
		//distanceFromBlob = camera.handleBlobs();
		detections = aprilTag.getDetections();
		numDetections = detections.size();
		telemetry.clear();
		telemetry.addData("Dets", detections);
		telemetry.addData("Voltage", hardwareMap.voltageSensor.get("Control Hub").getVoltage());


//        detections = aprilTag.getDetections();
//        if (!detections.isEmpty()) {
//            telemetry.addLine("HAS TAG");
//            for (AprilTagDetection detec : detections) {
//                det = detec;
//                if (absPosOfGryoStart == null) {
//                    det = detec;
//                    if (det.id == redTagId) {
//                        findAbsRobotPos(redTagPos);
//                        calculateGyroStartPos();
//                    }
//                    if (det.id == blueTagId) {
//                        findAbsRobotPos(blueTagPos);
//                        calculateGyroStartPos();
//                    }
//                }
//                if (!hasMotif) {
//          for(int i = 0; i < 3; i++) {
//              if (det.id == motifTagIds[i]) {
//                  greenBallPos = i;
//                  hasMotif = true;
//              }
//          }
//
//                }
//                if (gamepad1.dpad_left) {
//                    showTelem = true;
//                }
//                if (gamepad1.dpad_right) {
//                    showTelem = false;
//                }
//
//            }
//        }

		//can change to mechanum by changing line 247
		if (!isMovingToSetPos) {
			manualMechanumDrive();
		} else {
			updateAutoDrive();
		}

		if (gamepad1.x) {
			stopAutoMove();
		}
		//telemetry.addData("cameraTelem", showTelem);

		//telemetry.addData("Gyro X: ", robot.gyro.getPosition().x);
		//telemetry.addData("Gyro Y: ", robot.gyro.getPosition().y);
		//telemetry.addData("Gyro H: ", robot.gyro.getPosition().h);
		//telemetry.addData("knows gyro starting pos", absPosOfGryoStart != null);
		if (absPosOfGryoStart != null) {
			//telemetry.addData("startingPosGyro", pose2DtoString(absPosOfGryoStart));
			updateAbsoluteRobotPos();

		}
		if (absPosOfRobot != null) {
			//telemetry.addData("currentRobotPos", pose2DtoString(absPosOfRobot));
		}

		telemetry.addData("numDetections: ", numDetections);
		if (numDetections != 0) {
			det = detections.get(0);
		}
		if(det != null) {
			poseFtc = det.ftcPose;
		}
		if(poseFtc != null) {
			prevRange = poseFtc.range;
			telemetry.addData("range", poseFtc.range);
		}

		if (det != null) {
			if (det.id == redTagId || det.id == blueTagId) {
				if (det.id == redTagId) {
					currentTagPos = redTagPos;
				} else {
					currentTagPos = blueTagPos;
				}

//				if (gamepad1.dpad_up) {
//
//					double kP = 0.006; // tune
//
//					double turnPower = kP * det.ftcPose.bearing;
//					turnPower = Range.clip(turnPower, -0.3, 0.3);
//					if(gamepad1.a){
//						robot.turnTableRotator.setPower(turnPower);
//					} else {
//						turntable.setPower(0);
//					}
//				}
/*
				telemetry.addData("ID", det.id);
				telemetry.addData("Center", "(%.2f, %.2f)", det.center.x, det.center.y);

				telemetry.addData("Pose X", det.ftcPose.x);
				telemetry.addData("Pose Y", det.ftcPose.y);

 */
				telemetry.addData("Heading (deg)", poseFtc.yaw);
				telemetry.addData("Range", poseFtc.range);




				double tagHeadingRad = Math.toRadians(currentTagPos.getHeading(AngleUnit.DEGREES));
				double relX = det.ftcPose.x;
				double relY = det.ftcPose.y;

				double yField = currentTagPos.getX(DistanceUnit.INCH)
						+ relX * cos(tagHeadingRad)
						- relY * sin(tagHeadingRad);

				double xField = currentTagPos.getY(DistanceUnit.INCH)
						+ relX * sin(tagHeadingRad)
						+ relY * cos(tagHeadingRad);

				//telemetry.addData("field X", xField);
				//telemetry.addData("field Y", yField);

				//the x is the x, and the y is y

				double gx = robot.gyro.getPosition().x;  // raw gyro-based odometry
				double gy = robot.gyro.getPosition().y;
				double gh = robot.gyro.getPosition().h;


				double currentRobotHeadingField = det.ftcPose.yaw - currentTagPos.getHeading(AngleUnit.DEGREES);
				double gyroHeadingOrigin = currentRobotHeadingField - Math.toDegrees(gh);
//				telemetry.addData("current tag heading", currentTagPos.getHeading(AngleUnit.DEGREES));
//				telemetry.addData("yaw", det.ftcPose.yaw);
//				telemetry.addData("currentRobotHeading", currentRobotHeadingField);
//				telemetry.addData("gyroHeadingOrigin", gyroHeadingOrigin);


				//need to get this gh to be the offset between the gyro-h start and the field
				// Rotate gyro offsets into field space
				double[] gyroOriginCords = rotateCord(gx, gy, gyroHeadingOrigin, false);

				double gxField = gyroOriginCords[0];
				double gyField = gyroOriginCords[1];

//				telemetry.addData("gyroOriginX: ", gxField);
//				telemetry.addData("gyroOriginY: ", gyField);

				//if motif tag
			} else {
				for (int i = 0; i < 3; i++) {
					if (det.id == motifTagIds[i]) {
						greenBallPos = i;
					}
					hasMotif = true;

				}
			}
		}

		//telemetry.addData("Has motif: ", hasMotif);
		//telemetry.addData("motif: ", greenBallPos);


		//TODO: figure out where the launch zone is
		//either this should be based on the april tags, or just make sure that
		// the gyro is reset in the same spot every time
		if (gamepad1.a) {
			//Pose2D pos = getRelativeDiff(new Pose2D(DistanceUnit.INCH, 40, 40, AngleUnit.RADIANS, 0));
			//relativePosToTarget = pos;
			//NewPositionOfRobot pose = new NewPositionOfRobot(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.RADIANS));
			startAutoMove(new NewPositionOfRobot(0, 0, 0));
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
			//telemetry.addLine("Current Position  X: " + formattedX + "Y: " + formattedY);
		}
		//was using 0.7 and 0.85
//        if (gamepad2.a) {
//            robot.flyWheel.setPower(-0.5);
//            telemetry.addData("flywheel rate slow", robot.flyWheel.getVelocity(AngleUnit.DEGREES));
//        } else if (gamepad2.x) {
//            robot.flyWheel.setPower(-0.95);
//            telemetry.addData("flywheel rate fast", robot.flyWheel.getVelocity(AngleUnit.DEGREES));
//        } else {
//            robot.flyWheel.setPower(0);
//        }
		double stick = -gamepad2.right_stick_y;

		if (Math.abs(stick) > 0.08) {
			turntablePos += stick * 0.004;  // tune this constant
		}

		robot.turnTableRotator.setPosition(turntablePos);
		telemetry.addData("turntable pos", turntablePos);


			if(gamepad2.x && !isRamping){
			rampTime.reset();
			isRamping = true;
			}



		if(isRamping){
			//100 is the degrees per second it accelerates
			flyWheelVelo = (getVelo(rampTime.seconds()));
			if(det != null) {
					maxFlyWheelVelo = getAutomaticFlywheelVelo(poseFtc.range);
					flyWheelVelo = maxFlyWheelVelo;
				telemetry.addData("maxFlywheelVelo", maxFlyWheelVelo);
			}
			if(flyWheelVelo > maxFlyWheelVelo){
				flyWheelVelo = maxFlyWheelVelo;
				telemetry.addLine("at velo");
			}
		}

//			if(isRamping) {
//				flyWheelVelo = (getVelo(rampTime.seconds()));
//			} else if(flyWheelVelo != 1100 && flyWheelVelo != 1200 && flyWheelVelo != 1300){
//				flyWheelVelo = 1000;
//			}



//			if(gamepad2.b){
//				flyWheelVelo = 1200;
//			} else if (gamepad2.y){
//				flyWheelVelo = 1100;
//			} else if (gamepad2.a){
//				flyWheelVelo = 1300;
//			}

		if(flyWheelVelo > maxFlyWheelVelo){
			flyWheelVelo = maxFlyWheelVelo;
		}

		if (gamepad2.dpad_up) {
			maxFlyWheelVelo += 1;
		} else if (gamepad2.dpad_down) {
			maxFlyWheelVelo -= 1;
		}



		//}




//
//	if(gamepad1.dpad_left){
//		isManualFlywheel = false;
//	}
//	if(gamepad1.dpad_right){
//		isManualFlywheel = true;
//	}

		robot.flyWheel.setVelocity(flyWheelVelo);

//		if (gamepad1.left_bumper) {
//			targetFlyWheelVelo = -1400;//works for the corner peice close
//		} else if (gamepad1.right_bumper) {
//			targetFlyWheelVelo = -1900;
//		} else {
//			targetFlyWheelVelo = 0;
//		}

		//robot.flyWheel.setPower(-0.5);
		telemetry.addData("targetVelo", maxFlyWheelVelo);

		telemetry.addData("Actual Velo", robot.flyWheel.getVelocity());
		telemetry.addData("flywheel velo", flyWheelVelo);
		telemetry.addData("anglePos", robot.turnTableRotator.getPosition());


//		if(gamepad1.right_trigger > 0.5){
//			double range;
//			if(det != null) {
//				 range = det.ftcPose.range;
//			} else {
//				range = prevRange;
//			}
//			if(Math.abs(range - 50) < 10){
//				autoFlywheelVelo = -1400;
//			} else if(Math.abs(range - 80) < 20){
//				autoFlywheelVelo = -1550;
//			}else if (Math.abs(range - 120) < 20){
//				autoFlywheelVelo = -1800;
//			} else {
//				autoFlywheelVelo = -700;
//			}
//
//			robot.flyWheel.setVelocity(autoFlywheelVelo, AngleUnit.DEGREES);
//			robot.flyWheelRotator.setPosition(0.5);
//
//		}else {
//			robot.flyWheel.setVelocity(targetFlyWheelVelo, AngleUnit.DEGREES);
//			if (gamepad2.b) {
//				//telemetry.addData("b is pressed", "");
//				//more neg is higher
//				//more pos is lower
//				robot.flyWheelRotator.setPosition(0.6);
//			} else if (gamepad2.y) {
//				//telemetry.addData("y is pressed", "");
//				robot.flyWheelRotator.setPosition(farShootingPos);
//			} else if(gamepad2.x){
//			}
//
//
//			if(gamepad2.dpad_up){
//				farShootingPos += 0.01;
//			} else if (gamepad2.dpad_down) {
//				farShootingPos -= 0.01;
//			}
//		}





		//telemetry.addData("FlyWheelVelocity: ", robot.flyWheel.getVelocity(AngleUnit.DEGREES));
		//telemetry.addData("TargetFlywheelVelocity: ", targetFlyWheelVelo);
		//this checks if the difference is less than 1.5 spins per second
		if (Math.abs(robot.flyWheel.getVelocity() - flyWheelVelo) < 40 && robot.flyWheel.getVelocity() > 300) {
			robot.flyWheelLight.setPosition(green);
			rightVelo = true;
			isRamping = false;
		} else {
			rightVelo = false;
			robot.flyWheelLight.setPosition(red);
		}









		telemetry.addData("flyWheelPos1:", robot.flyWheelRotator.getPosition());
		 //telemetry.addData("varPos:", farShootingPos);


		telemetry.addData("cError", cError);
		telemetry.addData("cX", cX);
		telemetry.addData("cY", cY);
		telemetry.addData("cH", cH);


//		if (gamepad2.right_trigger > 0.5) {
//			robot.intake.setPower(0.95);
//			//uh I think this works 🥀
//		} else if (gamepad2.left_trigger > 0.5) {
//			robot.intake.setPower(-0.95);
//		} else {
//			robot.intake.setPower(0);
//		}

		//added methods to find the positions that work, once these are used then you can do set position ones
		// eventually, have two positions that are trapping a ball on the right and the left
		//can just toggle between these two positions

		boolean leftBumper = gamepad2.left_trigger > 0.5;
		boolean rightBumper = gamepad2.right_trigger > 0.5;

		if (rightBumper && !prevRightBumper && !servoAPressed && rightVelo) {
			servoAPressed = true;
			sort1Time.reset();
		}

		if (leftBumper && !prevLeftBumper && !servoBPressed && rightVelo) {
			servoBPressed = true;
			sort2Time.reset();
		}

		prevRightBumper = rightBumper;
		prevLeftBumper = leftBumper;


		if(servoAPressed){
			if(sort1Time.seconds() < 0.7){
				robot.sorter1.setPosition(s1high);
			} else {
				servoAPressed = false;
			}
		} else {
			robot.sorter1.setPosition(s1low);
		}

		if (servoBPressed) {
			if (sort2Time.seconds() < 0.7) {
				robot.sorter2.setPosition(s2low);
			} else {
				servoBPressed = false;
			}
		} else {
			robot.sorter2.setPosition(s2high);
		}


		if(gamepad2.dpad_right){
			hoodPosition += 0.01;
		} else if(gamepad2.dpad_left){
			hoodPosition -= 0.01;
		}

		robot.flyWheelRotator.setPosition(hoodPosition);
		telemetry.addData("hood position", hoodPosition);

		if(gamepad1.right_trigger > 0.5){
			intake();
		} else if(gamepad1.left_trigger > 0.5){
			outake();
		} else {
			stopIntake();
		}


//		if(gamepad2.dpad_left){
//			robot.sorter2.setPosition(s2low);
//		} else if (gamepad2.dpad_right){
//			robot.sorter2.setPosition(s2high);
//		}






//		if (gamepad2.dpad_right) {
//			robot.sorter1.setPosition(0.1);
//			//  telemetry.addData("sorterServoPos", robot.sorterServo.getPosition());
//		}
//
//		if (gamepad2.dpad_left) {
//			robot.sorter1.setPosition(0.6);
//			// telemetry.addData("sorterServoPos", robot.sorterServo.getPosition());
//		}


		telemetry.update();


	}

	public void intake(){
		robot.FLintake.setPower(0.5);
		robot.BLintake.setPower(0.5);
		robot.BRintake.setPower(0.5);
	}

	public void outake() {
		robot.FLintake.setPower(-0.5);
		robot.BLintake.setPower(-0.5);
		robot.BRintake.setPower(-0.5);
	}

	public void stopIntake(){
		robot.FLintake.setPower(0);
		robot.BLintake.setPower(0);
		robot.BRintake.setPower(0);
	}

	/**
	 * this method handles headless math and controls robot
	 */
	public void manualHeadlessDrive() {
		double y = -gamepad1.left_stick_y;
		double x = gamepad1.left_stick_x;
		double rx = -gamepad1.right_stick_x;

		SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();
		double botHeading = -pos.h;

		double[] rotCords = rotateCord(x, y, botHeading, false);
		double rotX = rotCords[0];
		double rotY = rotCords[1];

		double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
		robot.FLdrive.setPower(((rotY + rx + rotX) / denominator));
		robot.FRdrive.setPower(((rotY - rx - rotX) / denominator));
		robot.BLdrive.setPower(((rotY + rx - rotX) / denominator));
		robot.BRdrive.setPower(((rotY - rx + rotX) / denominator));
	}

	public void manualMechanumDrive() {
		double y = -gamepad1.left_stick_y;
		double x = gamepad1.left_stick_x;
		double rx = -gamepad1.right_stick_x;

		//headless
		robot.FLdrive.setPower(y + rx + x);
		robot.FRdrive.setPower(y - rx - x);
		robot.BLdrive.setPower(y + rx - x);
		robot.BRdrive.setPower(y - rx + x);
	}


	public void startAutoMove(NewPositionOfRobot target) {
		isMovingToSetPos = true;
		currentTarget = target;
	}

	public double getVelo(double seconds){
		if(seconds * (maxFlyWheelVelo / targetRampTime) < maxFlyWheelVelo) {
			return seconds * (maxFlyWheelVelo / targetRampTime);
		}

		if(maxFlyWheelVelo < MAXFLYWHEEL){
			return MAXFLYWHEEL;
		}
		return maxFlyWheelVelo;
	}

	public double getAutomaticFlywheelVelo(double distance){
		//y=0.000064734x^{4}-0.0216711x^{3}+2.61783x^{2}-129.19871x+3197.02016
		return 0.000064734 * Math.pow(distance, 4)
				- 0.0216711 * Math.pow(distance, 3)
				+ 2.61783 * Math.pow(distance, 2)
				- 129.19871 * distance // changed this from 129.1
				+ 3197.02016;
	}

	public boolean updateAutoDrive() {
		SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();
		currentPose.updateRealRobotPositions(pos);

		double error = currentPose.moveToSetPosition(currentTarget, robot);
		cError = error;
		// telemetry.addData("AutoMoving", true);

		// telemetry.addData("Error", error);

		if (Math.abs(error) < cTreshold) {
			stopAutoMove();
			return false;
		}
		return true;
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
		// telemetry.addData("AutoMove: ", "Stopped");
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


				double rotatedX = relX * cos(tagHeadingRad) - relY * sin(tagHeadingRad);
				double rotatedY = relX * sin(tagHeadingRad) + relY * cos(tagHeadingRad);

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
