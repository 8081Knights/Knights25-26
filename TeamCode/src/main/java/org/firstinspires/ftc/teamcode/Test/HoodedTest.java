package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
@Disabled
@TeleOp(name = "Hood Test")
public class HoodedTest extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();
    double pos = 0.5;

    @Override
    public void init() {
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        if(gamepad1.x){
            robot.intake.setPower(0.5);
            telemetry.addData("HI treyton", "JO momma");
        } else {
            robot.intake.setPower(0);
        }

        if(gamepad1.right_trigger > 0.5){
            robot.flyWheel.setPower(0.7);
        } else {
            robot.flyWheel.setPower(0);
        }

        if(gamepad1.a){
            pos += 0.02;
        }
        if(gamepad1.b){
            pos -= 0.02;
        }
        robot.flyWheelRotator2.setPosition(pos);


    }
}
