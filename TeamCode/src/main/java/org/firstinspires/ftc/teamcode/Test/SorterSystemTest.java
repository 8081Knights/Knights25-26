package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.BallState;
import org.firstinspires.ftc.teamcode.subsystems.Sorter;

@TeleOp(name = "sorter system test")
public class SorterSystemTest extends OpMode {
	Sorter sorterA = null;
	Sorter sorterB = null;
	Sorter sorterC = null;


	@Override
	public void init() {
		sorterA = new Sorter(hardwareMap, "sorterA", BallState.EMPTY, 0.7, 0.5, "ColorSensorA1", "ColorSensorA2");

	}

	//servoA - EH 2
	//

	@Override
	public void loop() {

		if (gamepad1.a && (sorterA.getServoTime() > Sorter.servoUpTime)) {
			sorterA.servoUp();
		}
		telemetry.addData("time", sorterA.getServoTime());
		telemetry.addData("up time", Sorter.servoUpTime);
		telemetry.addData("position", sorterA.servo.getPosition());

		sorterA.updateBallState();
		sorterA.updateFeedState();


	}
}
