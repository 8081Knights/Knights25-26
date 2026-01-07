package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "LEDTest")
public class LEDTest extends OpMode {

	//TODO: make the default be purple with the green rotating through slowly, then when it detects it blinks twice and sets the motif
	private Servo motifLight1 = null;
	private Servo motifLight2 = null;
	private Servo motifLight3 = null;
	private Servo flyWheelLight = null;

	double green = .5;
	double purple = .666;
	double red = .279;
	double black = 0;
	double yellow = .388;
	boolean atSpeed = false;
	int motifValue = 0;
	double t;
	int cycleIndex;

	ElapsedTime timer = new ElapsedTime();

	@Override
	public void init() {
		motifLight1 = hardwareMap.get(Servo.class, "motifLight1");
		motifLight2 = hardwareMap.get(Servo.class, "motifLight2");
		motifLight3 = hardwareMap.get(Servo.class, "motifLight3");
		flyWheelLight = hardwareMap.get(Servo.class, "flyWheelLight");

		timerStart();
	}

	public void loop() {
		t = timer.seconds();
		cycleIndex = (int) t % 3;

		// control what the detected motif is
		if (gamepad1.x) {
			motifValue = 1;
		} else if (gamepad1.a) {
			motifValue = 2;
		} else if (gamepad1.b) {
			motifValue = 3;
		} else {
			motifValue = 0;
		}

		// control if flywheel is good
		if (gamepad1.y) {
			atSpeed = true;
		} else {
			atSpeed = false;
		}

		// color motif lights
		switch (motifValue) {
			case 1:
				setMotifTo1();
				break;
			case 2:
				setMotifTo2();
				break;
			case 3:
				setMotifTo3();
				break;
			default:
				if (gamepad1.right_bumper) {
					blinkYellow(cycleIndex);
				} else if (gamepad1.left_bumper) {
					cycleMotifColors(cycleIndex);
				} else {
					setMotifToBlack();
				}
				break;
		}

		// color flywheel light
		if (atSpeed) {
			flyWheelLight.setPosition(green);
		} else {
			flyWheelLight.setPosition(red);
		}

	}

	public void timerStart() {
		timer.reset();
	}

	public void setMotifTo1() {
		motifLight1.setPosition(green);
		motifLight2.setPosition(purple);
		motifLight3.setPosition(purple);
	}

	public void setMotifTo2() {
		motifLight1.setPosition(purple);
		motifLight2.setPosition(green);
		motifLight3.setPosition(purple);
	}

	public void setMotifTo3() {
		motifLight1.setPosition(purple);
		motifLight2.setPosition(purple);
		motifLight3.setPosition(green);
	}

	public void setMotifToBlack() {
		motifLight1.setPosition(black);
		motifLight2.setPosition(black);
		motifLight3.setPosition(black);
	}

	public void setMotifToYellow() {
		motifLight1.setPosition(yellow);
		motifLight2.setPosition(yellow);
		motifLight3.setPosition(yellow);
	}

	public void cycleMotifColors(int cycleIndex) {
		switch (cycleIndex) {
			case 0:
				setMotifTo1();
				break;
			case 1:
				setMotifTo2();
				break;
			case 2:
				setMotifTo3();
				break;
			default:
				break;
		}
	}

	public void blinkYellow(int cycleIndex) {
		switch (cycleIndex) {
			case 0:
			case 1:
				setMotifToYellow();
				break;
			case 2:
				setMotifToBlack();
				break;
			default:
				break;
		}
	}
}
