package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="colorSensorTest")
public class ColorSensor extends OpMode {

    RevColorSensorV3 colorSensor;
    int red;
    int green;
    int blue;

    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
    }

    @Override
    public void loop() {
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();

        telemetry.addData("red", red);
        telemetry.addData("green", green);
        telemetry.addData("blue", blue);
        telemetry.update();

    }
}
