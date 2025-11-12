package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "flyWheel")
public class FlyWheelTest extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.flyWheel.setPower(-0.85);
            telemetry.addData("SHOOTING", " YAY");
        } else {
            robot.flyWheel.setPower(0);
        }
        telemetry.update();
    }


}
