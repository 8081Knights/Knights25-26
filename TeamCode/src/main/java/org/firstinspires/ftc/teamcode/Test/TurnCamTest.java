package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "turnCamTest")
public class TurnCamTest extends OpMode {
	Servo turntable;

	VisionPortal visionPortal;

	AprilTagProcessor aprilTag;

	List<AprilTagDetection> detections;

	AprilTagDetection bestDet = null;
	double bearing = 0;

	double ticksPerDegree = 4.71472;

	double pastServoPos;

	double targetServoPos;

	public DcMotorEx FRdrive = null;
	public DcMotorEx BRdrive = null;
	public DcMotorEx BLdrive = null;
	public DcMotorEx FLdrive = null;


	@Override
	public void init() {

		FLdrive = hardwareMap.get(DcMotorEx.class, "FLdrive");
		FRdrive = hardwareMap.get(DcMotorEx.class, "FRdrive");
		BLdrive = hardwareMap.get(DcMotorEx.class, "BLdrive");
		BRdrive = hardwareMap.get(DcMotorEx.class, "BRdrive");
		BLdrive.setDirection(DcMotorSimple.Direction.REVERSE);

		turntable = hardwareMap.get(Servo.class, "turntable");
		turntable.setPosition(0.5);
		pastServoPos = turntable.getPosition();
		targetServoPos = pastServoPos;


		aprilTag = new AprilTagProcessor.Builder()
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setDrawTagOutline(true)
				.build();

		visionPortal = new VisionPortal.Builder()
				.setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
				.addProcessor(aprilTag)
				.build();

	}

	@Override
	public void loop() {

		detections = aprilTag.getDetections();
		if (detections.isEmpty()) {
			bestDet = null;
		} else {
			for (AprilTagDetection det : detections) {
				//if (det.id == 24 || det.id == 20) {
				bestDet = det;
				//}
			}
		}

		if (bestDet != null) {
			bearing = bestDet.ftcPose.bearing;
		} else {
			bearing = 0;
		}

		telemetry.addData("bearing", bearing);
		telemetry.addData("tag", bestDet);

		double kP = 0.006; // tune

		double turnPower = kP * bearing;

		turnPower = Range.clip(turnPower, -0.3, 0.3);
		if (gamepad1.a) {
			double hi = Range.clip((targetServoPos + turnPower), 0, 1);
			turntable.setPosition(hi);
			telemetry.addData("Avelo", hi);
			pastServoPos = targetServoPos;
		} else if (gamepad1.b) {
			turntable.setPosition(Range.clip((turntable.getPosition() + 0.00003), 0, 1));
		} else if (gamepad1.y) {
			turntable.setPosition(Range.clip((turntable.getPosition() - 0.0003), 0, 1));
		}


		manualMechanumDrive();


//        if(gamepad1.a){
//            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turntable.setTargetPosition(degreesToTicks(360));
//            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turntable.setPower(0.6);
//        }else if (gamepad1.x){
//            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turntable.setTargetPosition(degreesToTicks(-360));
//            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turntable.setPower(0.6);
//        }

//        if(gamepad1.right_trigger > 0.5){
//            turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            turntable.setVelocity(400);
//        } else if (gamepad1.left_trigger > 0.5){
//            turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            turntable.setVelocity(-400);
//        } else if(turntable.getMode() == DcMotor.RunMode.RUN_USING_ENCODER){
//            turntable.setVelocity(0);
//        }

		telemetry.addData("velocity", turnPower);
		telemetry.addData("pos", turntable.getPosition());
		telemetry.addData("pastPos", pastServoPos);
		telemetry.addData("targetPos", targetServoPos);

		telemetry.update();
	}

	public int degreesToTicks(int degrees) {
		return (int) (degrees * ticksPerDegree);
	}
    /*
    public void turnToTag(){
        while (bearing > 100) {
            turn(-5, TURN_SPEED, x);
            telemetry.update();
        }
        while (bearing < 50) {
            turn(5, TURN_SPEED, x);
            telemetry.update();
        }
    }



    public void turn(int degrees, double speed, int X) {

        double inches = degrees * inchesPerDegree;
        inches = Math.abs(inches);
        if (degrees > 0) {
            continuousEncoderDrive(speed, -inches, inches, 10, X);
        } else {
            continuousEncoderDrive(speed, inches, -inches, 10, X);
        }
    }

     */


	public void manualMechanumDrive() {
		double y = -gamepad1.left_stick_y;
		double x = gamepad1.left_stick_x;
		double rx = -gamepad1.right_stick_x;

		//headless
		FLdrive.setPower(y + rx + x);
		FRdrive.setPower(y - rx - x);
		BLdrive.setPower(y + rx - x);
		BRdrive.setPower(y - rx + x);
	}
}
