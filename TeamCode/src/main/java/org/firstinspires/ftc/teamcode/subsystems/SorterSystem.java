package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;


public class SorterSystem {

	public HardwareSoftware hw;

	BallState Astate = BallState.EMPTY;
	BallState Bstate = BallState.EMPTY;
	BallState Cstate = BallState.EMPTY;

	ArrayList<RevColorSensorV3> sensorList = new ArrayList<>();

	ArrayList<Servo> servos = new ArrayList<>();


	ElapsedTime servo1Time = new ElapsedTime();
	ElapsedTime servo2Time = new ElapsedTime();
	ElapsedTime servo3Time = new ElapsedTime();

	double s1high = 0.63;
	double s1low = 0.31;

	double s2high = 0.95;
	double s2low = 0.63;


	double s3high = 0.70;
	double s3low = 0.30;


	public void setHardwareMap(HardwareSoftware hwa) {
		this.hw = hwa;
	}

	public void initColorSensors() {
		sensorList.add(hw.colorSensor1);
		sensorList.add(hw.colorSensor2);
		sensorList.add(hw.colorSensor3);
		sensorList.add(hw.colorSensor4);
		sensorList.add(hw.colorSensor5);
		sensorList.add(hw.colorSensor6);
	}


	public void feedGreen() {
		if (findServo(BallState.GREEN) != null) {
			findServo(BallState.GREEN).setPosition(getServoPos(findServo(BallState.GREEN), true));
		}
	}

	public void feedPurple() {

	}

	/**
	 * feeds all of the balls in no particular order
	 */
	public void feedAll() {

	}

	/**
	 * feeds all balls according to motif
	 *
	 * @param tagId current motif id
	 */
	public void feedMotif(int tagId) {

	}

	public double getServoPos(Servo servo, boolean isUp) {
		if (servo.equals(hw.sorterA)) {
			if (isUp) {
				return s1high;
			}
			return s1low;
		} else if (servo.equals(hw.sorterB)) {
			if (isUp) {
				return s2high;
			}
			return s2low;
		} else if (servo.equals(hw.sorterC)) {
			if (isUp) {
				return s3high;
			}
			return s3low;
		}
		return -2.0;
	}


	public void updateBallStates() {
		if (isPurple(hw.colorSensor1) || isPurple(hw.colorSensor2)) {
			Astate = BallState.PURPLE;
		} else if (isGreen(hw.colorSensor1) || (isGreen(hw.colorSensor2))) {
			Astate = BallState.GREEN;
		} else {
			Astate = BallState.EMPTY;
		}

		if (isPurple(hw.colorSensor3) || isPurple(hw.colorSensor4)) {
			Bstate = BallState.PURPLE;
		} else if (isGreen(hw.colorSensor3) || (isGreen(hw.colorSensor4))) {
			Bstate = BallState.GREEN;
		} else {
			Bstate = BallState.EMPTY;
		}

		if (isPurple(hw.colorSensor5) || isPurple(hw.colorSensor6)) {
			Cstate = BallState.PURPLE;
		} else if (isGreen(hw.colorSensor5) || (isGreen(hw.colorSensor6))) {
			Cstate = BallState.GREEN;
		} else {
			Cstate = BallState.EMPTY;
		}

	}


	public int getNumBalls() {
		int num = 0;
		if (!isEmpty(hw.colorSensor1) || !isEmpty(hw.colorSensor2)) {
			num++;
		}
		if (!isEmpty(hw.colorSensor3) || !isEmpty(hw.colorSensor4)) {
			num++;
		}
		if (!isEmpty(hw.colorSensor5) || !isEmpty(hw.colorSensor6)) {
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


	public Servo findServo(BallState state) {
		if (Astate == state) {
			return hw.sorterA;
		}
		if (Bstate == state) {
			return hw.sorterB;
		}
		if (Cstate == state) {
			return hw.sorterC;
		}
		return null;
	}


}
