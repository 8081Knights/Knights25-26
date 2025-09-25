package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name="DriveRed")
public class DriveRed extends OpMode {
    HardwareSoftware hw = new HardwareSoftware();

    @Override
    public void init() {

        hw.init(hardwareMap);

        hw.gyro().calibrateImu();
        hw.gyro().resetTracking();

    }

    double DS = 1;

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        SparkFunOTOS.Pose2D pos = hw.gyro().getPosition();

        double botHeading = -Math.toRadians(pos.h) + Math.PI;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        telemetry.addData("newX", rotX);
        telemetry.addData("newY", rotY);
        telemetry.addData("rx", rx);
        rotX = rotX * 1.1;  // Counteract imperfect strafing


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        hw.FLdrive().setPower(((rotY + rotX + rx) / denominator) * DS);
        hw.BLdrive().setPower(((rotY - rotX + rx) / denominator) * DS);
        hw.FRdrive().setPower(((rotY - rotX - rx) / denominator) * DS);
        hw.BRdrive().setPower(((rotY + rotX - rx) / denominator) * DS);

//        if (gamepad1.b) {
//            hw.Lucket().setPosition(.02);
//            hw.Rucket().setPosition(.09);
//        }
//        else if (gamepad1.a) {
//            hw.Lucket().setPosition(.64);
//            hw.Rucket().setPosition(.72);
//        }
//
//        if (gamepad1.dpad_up) {
//            hw.Linear().setTargetPosition(3000);
//            hw.Rinear().setTargetPosition(3000);
//            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Linear().setPower(1);
//            hw.Rinear().setPower(1);
//        }
//        if (gamepad1.dpad_left) {
//            hw.Linear().setTargetPosition(1500);
//            hw.Rinear().setTargetPosition(1500);
//            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Linear().setPower(1);
//            hw.Rinear().setPower(1);
//        }
//        if (gamepad1.dpad_down) {
//            hw.Linear().setTargetPosition(25);
//            hw.Rinear().setTargetPosition(25);
//            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Linear().setPower(1);
//            hw.Rinear().setPower(1);
//        }
//        if (gamepad1.right_bumper){
//            hw.InLinear().setPower(1);
//        }
//        else if (gamepad1.left_bumper){
//            hw.InLinear().setPower(-1);
//        }
//        else{
//            hw.InLinear().setPower(0);
//        }
//
//        if (gamepad1.right_trigger > .1) {
//            hw.Intake().setPower(gamepad1.right_trigger);
//        }
//        if (gamepad1.left_trigger > .1) {
//            hw.Intake().setPower(-gamepad1.left_trigger);
//        }
        if (gamepad1.share){
            hw.gyro().resetTracking();
        }
    }
}
