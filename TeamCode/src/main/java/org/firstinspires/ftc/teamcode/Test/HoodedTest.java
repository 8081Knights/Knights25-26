package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;

@Disabled
@TeleOp(name = "Hood Test")
public class HoodedTest extends OpMode {

	double pos = 0.5;
	int greenPosMotif = 0;
	int currentServo = 1;

	ArrayList<Integer> nums = new ArrayList<>();

	ArrayList<Servo> servos = new ArrayList<>();
	ArrayList<Servo> restServos = new ArrayList<>();
	ArrayList<Servo> activeServos = new ArrayList<>();

	double activeServoPos = 0.3;

	double restServoPos = 0;

	boolean isSorting = false;

	int velocity = -1000;

	int numServosMoved = 0;

	Servo sorter1;
	Servo sorter2;
	Servo sorter3;
	DcMotorEx flyWheel;
	DcMotorEx turnTableRotator;

	Servo flyWheelAdjuster = null;

	double hoodPos = 0.5;

	final double UP_POS = 0.7;
	final double DOWN_POS = 0.3;

	final double UP_TIME = 0.18;    // time to flick
	private ElapsedTime servoTimer = new ElapsedTime();


	@Override
	public void init() {
		sorter1 = hardwareMap.get(Servo.class, "sorter1");
		sorter2 = hardwareMap.get(Servo.class, "sorter2");
		sorter3 = hardwareMap.get(Servo.class, "sorter3");


		//flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
		//turnTableRotator = hardwareMap.get(DcMotorEx.class, "turntable");
		//flyWheelAdjuster = hardwareMap.get(Servo.class, "flyWheelAdjuster");


		servos.add(sorter1);
		servos.add(sorter2);
		servos.add(sorter3);
		restServos.clear();
		restServos.addAll(servos);
	}

	@Override
	public void loop() {
//		if (gamepad1.x) {
//			turnTableRotator.setPower(0.9);
//		} else if (gamepad1.y) {
//			turnTableRotator.setPower(-0.9);
//		} else {
//			turnTableRotator.setPower(0);
//		}


		//telemetry.addData("turntablething", turnTableRotator.getCurrentPosition());
		//1150 works for shooting from close
		//700 is waay to slow
		if (gamepad1.right_trigger > 0.5) {
			//robot.flyWheel.setPower(0.95);
			flyWheel.setVelocity(1800);
		} else if (gamepad1.left_trigger > 0.5) {
			// flyWheel.setVelocity(1150);
		} else if (gamepad1.left_bumper) {
			//flyWheel.setVelocity(velocity);
		} else {
			// flyWheel.setPower(0);
		}

		if (gamepad1.left_stick_y > 0.1) {
			velocity += 4;
		} else if (gamepad1.left_stick_y < -0.1) {
			velocity -= 4;
		}
		// telemetry.addData("flywheel custom velo: ", velocity);
		// telemetry.addData("Flywheel velo", flyWheel.getVelocity());
		// telemetry.addData("flywheel power", flyWheel.getPower());

		if (gamepad1.dpad_left) {
			hoodPos += 0.0001;
		} else if (gamepad1.dpad_right) {
			hoodPos -= 0.0001;
		}
		if (gamepad1.b) {
			//  flyWheelAdjuster.setPosition(hoodPos);
		}

		telemetry.addData("adjusterPos", hoodPos);
		//get position method is basically a get target position method, does not actually read the current position

		//rand number between 1 and 3
		if (gamepad1.dpad_up) {
			currentServo = (int) (Math.random() * 3) + 1;
		}

		if (gamepad1.dpad_down) {
			isSorting = true;
			restServos.remove(servos.get(currentServo - 1));
			activeServos.add(servos.get(currentServo - 1));
			servoTimer.reset();
		}

		if (isSorting) {

			if (servoTimer.seconds() > UP_TIME) {
				servoTimer.reset();
				currentServo++;
				numServosMoved++;
				if (currentServo > 3) {
					currentServo = 1;
				}
				restServos.remove(servos.get(currentServo - 1));
				activeServos.add(servos.get(currentServo - 1));

				if (numServosMoved > 2) {
					numServosMoved = 0;
					isSorting = false;
					activeServos.clear();
					restServos.clear();
					restServos.add(servos.get(0));
					restServos.add(servos.get(1));
					restServos.add(servos.get(2));
				}

			}
		}

		for (Servo s : restServos) {
			s.setPosition(DOWN_POS);
		}

		Servo s = servos.get(currentServo - 1);
		s.setPosition(UP_POS);

		telemetry.addData("currentServo", currentServo);
		if (greenPosMotif != 0) {
			telemetry.addData("greenPos", greenPosMotif);
		}

		telemetry.addData("sort1", sorter1.getPosition());
		telemetry.addData("sort2", sorter2.getPosition());
		telemetry.addData("sort3", sorter3.getPosition());
//
//
//
//        if (gamepad1.a) {
//            pos += 0.02;
//        }
//		 else if (gamepad1.b) {
//            pos -= 0.02;
//         }
//         pos = Math.min(1, pos);
//         //flyWheelRotator.setPosition(pos);
//         telemetry.addData("hood pos", pos);

	}
}

