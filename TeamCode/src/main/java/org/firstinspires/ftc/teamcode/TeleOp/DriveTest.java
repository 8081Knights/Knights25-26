package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
		if (gamepad1.x) {
			robot.BLdrive.setPower(0.6);
			robot.FLdrive.setPower(0.6);
			robot.BRdrive.setPower(0.6);
			robot.FRdrive.setPower(0.6);
		}

		if (gamepad1.a) {
			robot.BLdrive.setPower(0);
			robot.FLdrive.setPower(0);
			robot.BRdrive.setPower(0);
			robot.FRdrive.setPower(0);
		}

		if (gamepad2.a) {
			robot.FLdrive.setPower(0.4);
		} else {
			robot.FLdrive.setPower(0);
		}

		if (gamepad2.x) {
			robot.FRdrive.setPower(0.4);
		} else {
			robot.FRdrive.setPower(0);
		}

		if (gamepad2.b) {
			robot.BLdrive.setPower(0.4);
		} else {
			robot.BLdrive.setPower(0);
		}

		if (gamepad2.y) {
			robot.BRdrive.setPower(0.4);
		} else {
			robot.BRdrive.setPower(0);
		}
	}
}
