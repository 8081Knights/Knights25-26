package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;

enum FeedState {
	IDLE,
	SERVO_UP,
	SERVO_DOWN
}

enum BallState {
	PURPLE, GREEN, EMPTY
}

@TeleOp(name = "Servo + Color")
public class ServoAndColorSensorTest extends OpMode {

	ArrayList<Servo> servos = new ArrayList<>();
	ArrayList<Servo> activeServos = new ArrayList<>();

	Servo sorterA;
	Servo sorterB;
	Servo sorterC;


	final double UP_POS = 0.7;
	final double DOWN_POS = 0.0;
	final double UP_TIME = 0.6;    // time to flick

	FeedState feedState = FeedState.IDLE;
	int servoIndex = 0;
	ElapsedTime timer = new ElapsedTime();


	ArrayList<RevColorSensorV3> sensorList = new ArrayList<>();
	RevColorSensorV3 colorSensor1;
	RevColorSensorV3 colorSensor2;
	RevColorSensorV3 colorSensor3;
	RevColorSensorV3 colorSensor4;
	RevColorSensorV3 colorSensor5;
	RevColorSensorV3 colorSensor6;

	BallState Astate = BallState.EMPTY;
	BallState Bstate = BallState.EMPTY;
	BallState Cstate = BallState.EMPTY;


	@Override
	public void init() {
		initServos();
		initColorSensors();
	}

	public void initServos() {
		sorterA = hardwareMap.get(Servo.class, "sorter1");
		sorterB = hardwareMap.get(Servo.class, "sorter2");
		sorterC = hardwareMap.get(Servo.class, "sorter3");

		servos.add(sorterA);
		servos.add(sorterB);
		servos.add(sorterC);
	}

	public void initColorSensors() {
		colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor1");
		colorSensor1.setGain(2);
		colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor2");
		colorSensor2.setGain(2);
		colorSensor3 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor3");
		colorSensor3.setGain(2);
		colorSensor4 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor4");
		colorSensor4.setGain(2);
		colorSensor5 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor5");
		colorSensor5.setGain(2);
		colorSensor6 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor6");
		colorSensor6.setGain(2);

		sensorList.add(colorSensor1);
		sensorList.add(colorSensor2);
		sensorList.add(colorSensor3);
		sensorList.add(colorSensor4);
		sensorList.add(colorSensor5);
		sensorList.add(colorSensor6);
	}

	@Override
	public void loop() {
		updateStates();

		if ((gamepad1.a || gamepad1.b)) {
			if (feedState == FeedState.IDLE) {
				servoIndex = 0;
				feedState = FeedState.SERVO_UP;
				timer.reset();
			}

			if (gamepad1.a) {
				shootGreen();
			}
			if (gamepad1.b) {
				shootPurple();
			}

//            if(gamepad1.a && !activeServos.contains(sorterA)){
//                activeServos.add(sorterA);
//            }
//            if(gamepad1.b && !activeServos.contains(sorterB)){
//                activeServos.add(sorterB);
//            }
//            if(gamepad1.x && !activeServos.contains(sorterC)){
//                activeServos.add(sorterC);
//            }
		}

//        if (gamepad1.dpad_down && feedState == FeedStateAI.IDLE) {
//            servoIndex = 0;
//            feedState = FeedStateAI.SERVO_UP;
//            timer.reset();
//        }

		telemetry.addData("servo state", feedState);
		telemetry.addData("servoIndex", servoIndex);
		switch (feedState) {

			case SERVO_UP:
				activeServos.get(servoIndex).setPosition(UP_POS);
				if (timer.seconds() > UP_TIME) {
					feedState = FeedState.SERVO_DOWN;
					timer.reset();
				}
				break;

			case SERVO_DOWN:
				activeServos.get(servoIndex).setPosition(DOWN_POS);
				if (timer.seconds() > 0.12) {
					servoIndex++;
					if (servoIndex >= activeServos.size()) {
						feedState = FeedState.IDLE;
						activeServos.clear();
					} else {
						feedState = FeedState.SERVO_UP;
					}
					timer.reset();
				}
				break;

			case IDLE:
				for (Servo s : servos) {
					s.setPosition(DOWN_POS);
				}
				break;
		}


		telemetry.addData("sort1", sorterA.getPosition());
		telemetry.addData("sort2", sorterB.getPosition());
		telemetry.addData("sort3", sorterC.getPosition());
		telemetry.addData("ballA", Astate.name());
		telemetry.addData("ballB", Bstate.name());
		telemetry.addData("ballC", Cstate.name());

	}

	public void updateStates() {
		if (isPurple(colorSensor1) || isPurple(colorSensor2)) {
			Astate = BallState.PURPLE;
		} else if (isGreen(colorSensor1) || (isGreen(colorSensor2))) {
			Astate = BallState.GREEN;
		} else {
			Astate = BallState.EMPTY;
		}

		if (isPurple(colorSensor3) || isPurple(colorSensor4)) {
			Bstate = BallState.PURPLE;
		} else if (isGreen(colorSensor3) || (isGreen(colorSensor4))) {
			Bstate = BallState.GREEN;
		} else {
			Bstate = BallState.EMPTY;
		}

		if (isPurple(colorSensor5) || isPurple(colorSensor6)) {
			Cstate = BallState.PURPLE;
		} else if (isGreen(colorSensor5) || (isGreen(colorSensor6))) {
			Cstate = BallState.GREEN;
		} else {
			Cstate = BallState.EMPTY;
		}

	}

	public void shootGreen() {
		if (getNumBalls() == 0) {
			return;
		}
		if (Astate == BallState.GREEN) {
			activeServos.add(sorterA);
		}
		if (Bstate == BallState.GREEN) {
			activeServos.add(sorterB);
		}
		if (Cstate == BallState.GREEN) {
			activeServos.add(sorterC);
		}

	}


	public void shootPurple() {
		if (getNumBalls() == 0) {
			return;
		}
		if (Astate == BallState.PURPLE) {
			activeServos.add(sorterA);
		}
		if (Bstate == BallState.PURPLE) {
			activeServos.add(sorterB);
		}
		if (Cstate == BallState.PURPLE) {
			activeServos.add(sorterC);
		}

	}


	public int getNumBalls() {
		int num = 0;
		if (!isEmpty(colorSensor1) || !isEmpty(colorSensor2)) {
			num++;
		}
		if (!isEmpty(colorSensor3) || !isEmpty(colorSensor4)) {
			num++;
		}
		if (!isEmpty(colorSensor5) || !isEmpty(colorSensor6)) {
			num++;
		}

		return num;
	}

	public boolean isEmpty(RevColorSensorV3 sensor) {
		NormalizedRGBA colors = sensor.getNormalizedColors();

		float r = colors.red;
		float g = colors.green;
		float b = colors.blue;

		int red = (int) (r * 255);
		int green = (int) (g * 255);
		int blue = (int) (b * 255);
		return (blue <= 2 && red <= 2 && green <= 2);
	}

	public boolean isPurple(RevColorSensorV3 sensor) {
		NormalizedRGBA colors = sensor.getNormalizedColors();
		float r = colors.red;
		float g = colors.green;
		float b = colors.blue;

		int red = (int) (r * 255);
		int green = (int) (g * 255);
		int blue = (int) (b * 255);
		return (blue >= 25 && red >= 10 && green <= 15);
	}

	public boolean isGreen(RevColorSensorV3 sensor) {
		NormalizedRGBA colors = sensor.getNormalizedColors();

		float r = colors.red;
		float g = colors.green;
		float b = colors.blue;

		int red = (int) (r * 255);
		int green = (int) (g * 255);
		int blue = (int) (b * 255);
		return (green >= 25 && blue <= 30 && red <= 10);

	}

}
