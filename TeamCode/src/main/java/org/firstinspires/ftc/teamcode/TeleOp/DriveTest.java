package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name="DriveTest")
public class DriveTest extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();
    public void init(){
        robot.init(hardwareMap);
    }
    public void loop(){
        if (gamepad1.x) {
            robot.BLdrive.setPower(0.7);
        }
        if (gamepad1.a) {
            robot.FLdrive.setPower(0.7);
        }
        if (gamepad1.y) {
            robot.BRdrive.setPower(0.7);
        }
        if (gamepad1.b) {
            robot.FRdrive.setPower(0.7);
        }
    }
}
