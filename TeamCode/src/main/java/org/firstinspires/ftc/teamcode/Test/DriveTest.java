package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name = "DriveTest")
public class DriveTest extends OpMode {
	//Class to test drive subsystem
	//TEST, NOT MATCHES
	//HardwareSoftware robot = new HardwareSoftware();
	public DcMotorEx FRdrive = null;
	public DcMotorEx BRdrive = null;
	public DcMotorEx BLdrive = null;
	public DcMotorEx FLdrive = null;

	public void init() {
		//init(hardwareMap);
		FLdrive = hardwareMap.get(DcMotorEx.class, "FLdrive");
		FRdrive = hardwareMap.get(DcMotorEx.class, "FRdrive");
		BLdrive = hardwareMap.get(DcMotorEx.class, "BLdrive");
		BRdrive = hardwareMap.get(DcMotorEx.class, "BRdrive");
	}

	public void loop() {
/*		if(gamepad1.x){
			FLdrive.setVelocity(200, AngleUnit.DEGREES);
		}
		if (gamepad1.x) {
			FRdrive.setVelocity(200, AngleUnit.DEGREES);
		}
		if(gamepad1.x){
			BLdrive.setVelocity(200, AngleUnit.DEGREES);
		}
		if(gamepad1.x){
			BRdrive.setVelocity(200, AngleUnit.DEGREES);
		}*/

		if (gamepad1.x) {
			FLdrive.setPower(0.9);
		}
		if (gamepad1.x) {
			FRdrive.setPower(0.9);
		}
		if (gamepad1.x) {
			BLdrive.setPower(0.9);
		}
		if (gamepad1.x) {
			BRdrive.setPower(0.9);
		}

		telemetry.addData("FL", FLdrive.getVelocity(AngleUnit.DEGREES));
		telemetry.addData("FR", FRdrive.getVelocity(AngleUnit.DEGREES));
		telemetry.addData("BL", BLdrive.getVelocity(AngleUnit.DEGREES));
		telemetry.addData("BR", BRdrive.getVelocity(AngleUnit.DEGREES));


		if (gamepad1.a) {
			BLdrive.setPower(0);
			FLdrive.setPower(0);
			BRdrive.setPower(0);
			FRdrive.setPower(0);
		}
		double pow = 0.4;
		if (gamepad2.a) {
			FLdrive.setPower(pow);
		} else {
			FLdrive.setPower(0);
		}

		if (gamepad2.x) {
			FRdrive.setPower(pow);
		} else {
			FRdrive.setPower(0);
		}

		if (gamepad2.b) {
			BLdrive.setPower(pow);
		} else {
			BLdrive.setPower(0);
		}

		if (gamepad2.y) {
			BRdrive.setPower(pow);
		} else {
			BRdrive.setPower(0);
		}
	}
}
