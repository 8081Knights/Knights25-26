package org.firstinspires.ftc.teamcode.Test;

import static org.firstinspires.ftc.teamcode.HelperMethods.rotateCord;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
@TeleOp(name = "GyroTest")
public class GyroTest extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initGyro();
    }

    @Override
    public void loop() {
    manualHeadlessDrive();
    telemetry.addData("g-x", robot.gyro.getPosition().x);
    telemetry.addData("g-y", robot.gyro.getPosition().y);
    telemetry.addData("g-h", robot.gyro.getPosition().h);

        if (gamepad1.b) {
            robot.gyro.resetTracking();
        }

    }

    public void manualHeadlessDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();
        double botHeading = -pos.h;

        double[] rotCords = rotateCord(x, y, botHeading, false);
        double rotX = rotCords[0];
        double rotY = rotCords[1];

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        robot.FLdrive.setPower(((rotY + rotX + rx) / denominator));
        robot.BLdrive.setPower(((rotY - rotX + rx) / denominator));
        robot.FRdrive.setPower(((rotY - rotX - rx) / denominator));
        robot.BRdrive.setPower(((rotY + rotX - rx) / denominator));
    }
}
