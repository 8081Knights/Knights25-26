
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

import java.util.ArrayList;

public class HardwareSoftware {

	private HardwareMap hw = null;


	//wheels
	public DcMotorEx FRdrive = null;
	public DcMotorEx BRdrive = null;
	public DcMotorEx BLdrive = null;
	public DcMotorEx FLdrive = null;

	public DcMotorEx flyWheel = null;

	public Servo flyWheelRotator = null;
	public Servo turnTableRotator = null;

	public Servo sorterA = null;
	public Servo sorterB = null;

	public Servo sorterC = null;

	public CRServo FRintake = null;
	public CRServo FLintake = null;
	public CRServo BRintake = null;
	public CRServo BLintake = null;


	public SparkFunOTOS gyro;

	public Servo flyWheelLight = null;


	public RevColorSensorV3 colorSensor1;
	public RevColorSensorV3 colorSensor2;
	public RevColorSensorV3 colorSensor3;
	public RevColorSensorV3 colorSensor4;
	public RevColorSensorV3 colorSensor5;
	public RevColorSensorV3 colorSensor6;

	public Sorter sorterTJ;


	/**
	 * initializes the motors and servos
	 *
	 * @param ahw
	 */
	public void init(HardwareMap ahw) {

		hw = ahw;

		FLdrive = hw.get(DcMotorEx.class, "FLdrive");
		FRdrive = hw.get(DcMotorEx.class, "FRdrive");
		BLdrive = hw.get(DcMotorEx.class, "BLdrive");
		BRdrive = hw.get(DcMotorEx.class, "BRdrive");


		flyWheelRotator = hw.get(Servo.class, "hoodAdjuster");
		turnTableRotator = hw.get(Servo.class, "turntable");
		flyWheel = hw.get(DcMotorEx.class, "flywheel");
		sorterA = hw.get(Servo.class, "sorterA");
		sorterB = hw.get(Servo.class, "sorterB");
		sorterC = hw.get(Servo.class, "sorterC");

		FRintake = hw.get(CRServo.class, "FRintake");
		FLintake = hw.get(CRServo.class, "FLintake");
		BRintake = hw.get(CRServo.class, "BRintake");
		BLintake = hw.get(CRServo.class, "BLintake");

		BLintake.setDirection(DcMotorSimple.Direction.REVERSE);


		flyWheelLight = hw.get(Servo.class, "flyWheelLight");


		gyro = hw.get(SparkFunOTOS.class, "gyro");


		colorSensor1 = hw.get(RevColorSensorV3.class, "ColorSensor1");
		colorSensor1.setGain(2);
		colorSensor2 = hw.get(RevColorSensorV3.class, "ColorSensor2");
		colorSensor2.setGain(2);
		colorSensor3 = hw.get(RevColorSensorV3.class, "ColorSensor3");
		colorSensor3.setGain(2);
		colorSensor4 = hw.get(RevColorSensorV3.class, "ColorSensor4");
		colorSensor4.setGain(2);
		colorSensor5 = hw.get(RevColorSensorV3.class, "ColorSensor5");
		colorSensor5.setGain(2);
		colorSensor6 = hw.get(RevColorSensorV3.class, "ColorSensor6");
		colorSensor6.setGain(2);


		FLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		BRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		FRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
		BLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		flyWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);


		FLdrive.setDirection(DcMotorEx.Direction.FORWARD);
		BLdrive.setDirection(DcMotorEx.Direction.REVERSE);
		FRdrive.setDirection(DcMotorEx.Direction.FORWARD);
		BRdrive.setDirection(DcMotorEx.Direction.FORWARD);

		FLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		BRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		FRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		BLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


	}

	/**
	 * initialize the gyro stuff
	 */
	public void initGyro() {
		gyro.calibrateImu();
		gyro.resetTracking();

		gyro.setLinearUnit(DistanceUnit.INCH);
		gyro.setAngularUnit(AngleUnit.RADIANS);

		SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-6.186, 0.7, 0);
		gyro.setOffset(offset);
		gyro.setLinearScalar(1.0);
		gyro.setAngularScalar(1.0);
		gyro.calibrateImu();
		gyro.resetTracking();
		SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
		gyro.setPosition(currentPosition);
	}

	/**
	 * initialize the gyro and add current spot on field to make the positions always the same
	 *
	 * @param x
	 * @param y
	 * @param h
	 */
	public void initGyro(double x, double y, double h) {
		gyro.calibrateImu();
		gyro.resetTracking();

		gyro.setLinearUnit(DistanceUnit.INCH);
		gyro.setAngularUnit(AngleUnit.RADIANS);

		SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-6.186, 0.7, 0);
		gyro.setOffset(offset);
		gyro.setLinearScalar(1.0);
		gyro.setAngularScalar(1.0);
		gyro.calibrateImu();
		gyro.resetTracking();
		SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, h);
		gyro.setPosition(currentPosition);
	}
}
