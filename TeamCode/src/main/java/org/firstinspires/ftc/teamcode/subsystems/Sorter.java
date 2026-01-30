package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sorter {

	public enum FeedState {
		SERVO_UP,
		SERVO_DOWN
	}


	BallState ballState;
	FeedState servoState;
	ElapsedTime servoTimer = new ElapsedTime();

	double highPos;
	double lowPos;

	Servo servo;
	RevColorSensorV3 sensor1;
	RevColorSensorV3 sensor2;


	/**
	 * @param hw          hardware map
	 * @param servoName   configuration name for the servo
	 * @param state       starting state of the sorter(does not matter much)
	 * @param highPos     position of servo at the top of motion
	 * @param lowPos      position of servo when it is at rest
	 * @param sensor1Name configuration name for the first sensor
	 * @param sensor2Name configuration name for the second sensor
	 */
	public Sorter(HardwareMap hw, String servoName, BallState state, double highPos, double lowPos, String sensor1Name, String sensor2Name) {
		ballState = state;
		this.highPos = highPos;
		this.lowPos = lowPos;
		sensor1 = hw.get(RevColorSensorV3.class, sensor1Name);
		sensor2 = hw.get(RevColorSensorV3.class, sensor2Name);
		servo = hw.get(Servo.class, servoName);
		servoTimer.reset();
	}

	public BallState getBallState() {
		return ballState;
	}

	public FeedState getServoState() {
		return servoState;
	}

	public void feedBall() {
		servo.setPosition(highPos);
	}

	public void moveServo() {
		if (servoState == FeedState.SERVO_UP && Math.abs(servo.getPosition() - highPos) > 0.01) {
			servo.setPosition(highPos);
		}
		if (servoState == FeedState.SERVO_DOWN && Math.abs(servo.getPosition()) - lowPos > 0.01) {
			servo.setPosition(lowPos);
		}
	}

	public void updateFeedState() {
		if (Math.abs(servo.getPosition() - highPos) < 0.01) {
			servoState = FeedState.SERVO_UP;
		}
		if (Math.abs(servo.getPosition()) - lowPos < 0.01) {
			servoState = FeedState.SERVO_DOWN;
		}
	}

	public void updateBallState() {
		if (isGreen(sensor1) || isGreen(sensor2)) {
			ballState = BallState.GREEN;
		} else if (isPurple(sensor1) || isPurple(sensor2)) {
			ballState = BallState.PURPLE;
		} else {
			ballState = BallState.EMPTY;
		}
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
