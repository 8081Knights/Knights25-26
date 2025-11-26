package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.HelperMethods.normalizeAngle;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.Arrays;

public class CurrentRobotPose {
	HardwareSoftware robotHardwaremap;
	public double realRobotX, realRobotY, realRobotHeading;
	public double gyX, gyY, gyR;

	public double initX, initY, initZ;

	/**
	 * Initializes the robot's pose.
	 *
	 * @param hwMap The hardware map.
	 * @param inX   The initial x-coordinate.
	 * @param inY   The initial y-coordinate.
	 * @param inz   The initial z-coordinate.
	 */
	public void init(HardwareSoftware hwMap, double inX, double inY, double inz) {
		this.robotHardwaremap = hwMap;
		this.initX = inX;
		this.initY = inY;
		this.initZ = inz;
	}

	/**
	 * Updates the robot's real positions based on gyroscope values.
	 *
	 * @param gyroValue The current gyroscope position.
	 */
	public void updateRealRobotPositions(SparkFunOTOS.Pose2D gyroValue) {
		gyX = gyroValue.x;
		gyY = gyroValue.y;
		gyR = gyroValue.h;

		realRobotX = initX + gyX;
		realRobotY = initY + gyY;
		realRobotHeading = initZ + normalizeAngle(gyR);
	}

	/**
	 * Moves the robot to a set position and returns the current error.
	 *
	 * @param setPose The target position and rotation.
	 * @return The current error between the robot's position and the target position.
	 */
	public double moveToSetPosition(NewPositionOfRobot setPose, HardwareSoftware robot) {
		double currentError = 0;
		double powY, powX, rx = 0;
		double powdY, powdX;

		powdX = setPose.newx - realRobotX;
		powdY = setPose.newy - realRobotY;

		if (powdX > 3 || powdX < -3) {
			powX = Math.signum(powdX);
		} else {
			powX = powdX / 3;
		}

		if (powdY > 3 || powdY < -3) {
			powY = Math.signum(powdY);
		} else {
			powY = powdY / 3;
		}

		double[] altAngles = new double[3];
		double[] diffAngles = new double[3];

		altAngles[0] = setPose.newRotation - 2 * Math.PI;
		altAngles[1] = setPose.newRotation;
		altAngles[2] = setPose.newRotation + 2 * Math.PI;

		for (int i = 0; i < 3; ++i) {
			diffAngles[i] = altAngles[i] - realRobotHeading;
		}

		Arrays.sort(diffAngles);

		int goodindex = 0;

		if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[0])) {
			goodindex = 1;
		}
		if (Math.abs(diffAngles[2]) < Math.abs(diffAngles[0])) {
			goodindex = 2;
		}
		if (Math.abs(diffAngles[2]) < Math.abs(diffAngles[1])) {
			goodindex = 2;
		}
		if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[2])) {
			goodindex = 1;
		}

		rx = diffAngles[goodindex];

//            telemetry.addData("diffIn0", diffAngles[0]);
//            telemetry.addData("diffIn1", diffAngles[1]);
//            telemetry.addData("diffIn2", diffAngles[2]);


		double dx = setPose.newx - realRobotX;
		double dy = setPose.newy - realRobotY;

		double botHeading = -realRobotHeading;
		double realSetX = dx * cos(botHeading) - dy * sin(botHeading);
		double realSetY = dx * sin(botHeading) + dy * cos(botHeading);

//            telemetry.addData("powx", powX);
//            telemetry.addData("powy", powY);
//            telemetry.addData("realSetX", realSetX);
//            telemetry.addData("realSetY", realSetY);
//            telemetry.addData("rx", rx);

		//basically just does headless until it gets to the right position
		double denominator = Math.max(Math.abs(powY) + Math.abs(powX) + Math.abs(rx), 1);

		robot.FLdrive.setPower(((-realSetY - realSetX - rx) / denominator) * setPose.speed);
		robot.BLdrive.setPower(((-realSetY + realSetX - rx) / denominator) * setPose.speed);
		robot.FRdrive.setPower(((-realSetY + realSetX + rx) / denominator) * setPose.speed);
		robot.BRdrive.setPower(((-realSetY - realSetX + rx) / denominator) * setPose.speed);
		currentError = Math.abs(dx) + Math.abs(dy) + Math.abs(rx);
		return currentError;
	}

}
