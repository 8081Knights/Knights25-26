package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
@Disabled
@TeleOp(name = "flyWheel")
public class FlyWheelTest extends OpMode {
	//flywheel test class
	//TEST, NOT MATCHES
	DcMotorEx flyWheel = null;
	@Override
	public void init() {
		flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
	}

	@Override
	public void loop() {
		if (gamepad1.a) {
			flyWheel.setVelocity(-1300);
			telemetry.addData("SHOOTING", " YAY");
			telemetry.addData("velo", flyWheel.getVelocity());
		} else {
			flyWheel.setPower(0);
		}
		telemetry.update();
	}


}
