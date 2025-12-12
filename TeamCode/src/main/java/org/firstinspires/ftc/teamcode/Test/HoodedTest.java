package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "Hood Test")
public class HoodedTest extends OpMode {
	private DcMotorEx flyWheel = null;

	private Servo flyWheelRotator = null;
	private DcMotorEx turnTableRotator = null;
	double pos = 0.5;

	@Override
	public void init() {
		flyWheelRotator = hardwareMap.get(Servo.class, "flyWheelRotator");
		turnTableRotator = hardwareMap.get(DcMotorEx.class, "turnTableRotator");
		flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

	}

	@Override
	public void loop() {

		if (gamepad1.x) {
			turnTableRotator.setPower(0.3);
		} else if (gamepad1.y) {
			turnTableRotator.setPower(-0.3);
		} else {
			turnTableRotator.setPower(0);
		}

		if (gamepad1.right_trigger > 0.5) {
			flyWheel.setPower(-0.9);
			telemetry.addData("Flywheel velo", flyWheel.getVelocity());
			telemetry.addData("flywheel power", flyWheel.getPower());
		} else {
			flyWheel.setPower(0);
		}

		if (gamepad1.a) {
			pos += 0.02;
		}
		if (gamepad1.b) {
			pos -= 0.02;
		}
		flyWheelRotator.setPosition(pos);


	}
}
