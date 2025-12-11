package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "DriveTest")
public class DriveTest extends OpMode {
	//Class to test drive subsystem
	//TEST, NOT MATCHES
	HardwareSoftware robot = new HardwareSoftware();

	public void init() {
		robot.init(hardwareMap);
	}

	public void loop() {
/*		if(gamepad1.x){
			robot.FLdrive.setVelocity(200, AngleUnit.DEGREES);
		}
		if (gamepad1.x) {
			robot.FRdrive.setVelocity(200, AngleUnit.DEGREES);
		}
		if(gamepad1.x){
			robot.BLdrive.setVelocity(200, AngleUnit.DEGREES);
		}
		if(gamepad1.x){
			robot.BRdrive.setVelocity(200, AngleUnit.DEGREES);
		}*/

		if(gamepad1.x){
			robot.FLdrive.setPower(0.9);
		}
		if (gamepad1.x) {
			robot.FRdrive.setPower(0.9);
		}
		if(gamepad1.x){
			robot.BLdrive.setPower(0.9);
		}
		if(gamepad1.x){
			robot.BRdrive.setPower(0.9);
		}

		telemetry.addData("FL", robot.FLdrive.getVelocity(AngleUnit.DEGREES));
		telemetry.addData("FR", robot.FRdrive.getVelocity(AngleUnit.DEGREES));
		telemetry.addData("BL", robot.BLdrive.getVelocity(AngleUnit.DEGREES));
		telemetry.addData("BR", robot.BRdrive.getVelocity(AngleUnit.DEGREES));


		if (gamepad1.a) {
			robot.BLdrive.setPower(0);
			robot.FLdrive.setPower(0);
			robot.BRdrive.setPower(0);
			robot.FRdrive.setPower(0);
		}
double pow = 0.4;
		if (gamepad2.a) {
			robot.FLdrive.setPower(pow);
		} else {
			robot.FLdrive.setPower(0);
		}

		if (gamepad2.x) {
			robot.FRdrive.setPower(pow);
		} else {
			robot.FRdrive.setPower(0);
		}

		if (gamepad2.b) {
			robot.BLdrive.setPower(pow);
		} else {
			robot.BLdrive.setPower(0);
		}

		if (gamepad2.y) {
			robot.BRdrive.setPower(pow);
		} else {
			robot.BRdrive.setPower(0);
		}
	}
}
