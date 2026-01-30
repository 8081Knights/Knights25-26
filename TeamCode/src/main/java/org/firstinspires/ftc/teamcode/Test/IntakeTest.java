package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake test")
public class IntakeTest extends OpMode {
	CRServo intake = null;

	@Override
	public void init() {
		intake = hardwareMap.get(CRServo.class, "intake");
	}

	@Override
	public void loop() {
		if (gamepad1.a) {
			intake.setPower(0.5);
		} else if (gamepad1.b) {
			intake.setPower(-0.5);
		} else {
			intake.setPower(0);
		}
	}


}