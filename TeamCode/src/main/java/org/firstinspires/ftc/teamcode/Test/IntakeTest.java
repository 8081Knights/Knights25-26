package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake test")
public class IntakeTest extends OpMode {
    DcMotorEx intake = null;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            intake.setPower(0.9);
        } else if(gamepad1.b){
            intake.setPower(-0.9);
        } else {
            intake.setPower(0);
        }
    }


}
