package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "LEDTest")
public class LEDTest extends OpMode {
	private Servo motifLight1 = null;
	private Servo motifLight2 = null;
	private Servo motifLight3 = null;
	private Servo flyWheelLight = null;

	double green = .5;
	double purple = .666;
	double red = .279;
	double black = 0;
	boolean atSpeed = false;
	int motifValue = 0;

	@Override
	public void init() {
		motifLight1 = hardwareMap.get(Servo.class, "motifLight1");
		motifLight2 = hardwareMap.get(Servo.class, "motifLight2");
		motifLight3 = hardwareMap.get(Servo.class, "motifLight3");
		flyWheelLight = hardwareMap.get(Servo.class, "flyWheelLight");
	}

	public void loop() {
		// control motif value
		if (gamepad1.x) {
			motifValue = 1;
		} else if (gamepad1.a) {
			motifValue = 2;
		} else if (gamepad1.b) {
			motifValue = 3;
		} else {
			motifValue = 0;
		}

		// control flywheel value
		if (gamepad1.y) {
			atSpeed = true;
		} else {
			atSpeed = false;
		}

		// color motif lights
		switch (motifValue) {
			case 1:
				motifLight1.setPosition(green);
				motifLight2.setPosition(purple);
				motifLight3.setPosition(purple);
				break;
			case 2:
				motifLight1.setPosition(purple);
				motifLight2.setPosition(green);
				motifLight3.setPosition(purple);
				break;
			case 3:
				motifLight1.setPosition(purple);
				motifLight2.setPosition(purple);
				motifLight3.setPosition(green);
				break;
			default:
				motifLight1.setPosition(black);
				motifLight2.setPosition(black);
				motifLight3.setPosition(black);
				break;
		}

		// color flywheel light
		if (atSpeed) {
			flyWheelLight.setPosition(green);
		} else {
			flyWheelLight.setPosition(red);
		}

	}
}
