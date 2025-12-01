package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="colorSensorTest")
public class ColorSensor extends OpMode {

    RevColorSensorV3 colorSensor;
    NormalizedRGBA colors;

    float r;
    float g;
    float b;

    int red;
    int green;
    int blue;

    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensor");
        colorSensor.setGain(2);
    }

    @Override
    public void loop() {
        colors = colorSensor.getNormalizedColors();

         r = colors.red;
         g = colors.green;
         b = colors.blue;

         red = (int)(r * 255);
        green = (int)(g * 255);
        blue = (int)(b * 255);
        if (green >= 25 && blue <= 30 && red <= 10){
            telemetry.addData("Is Green", true);
        }
        if (blue >= 25 && red >= 10 && green <= 15) {
            telemetry.addData("Is Purple", true);
        }
        telemetry.addData("green", green);
        telemetry.addData("blue", blue);
        telemetry.update();
    }
}

