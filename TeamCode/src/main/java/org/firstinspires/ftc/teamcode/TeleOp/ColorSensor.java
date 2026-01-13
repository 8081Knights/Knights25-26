package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import java.util.ArrayList;
enum BallState {
    PURPLE, GREEN, EMPTY
}
@TeleOp(name="colorSensorTest")
public class ColorSensor extends OpMode {

    ArrayList<RevColorSensorV3> sensorList = new ArrayList<>();
    RevColorSensorV3 colorSensor1;
    RevColorSensorV3 colorSensor2;
    RevColorSensorV3 colorSensor3;
    RevColorSensorV3 colorSensor4;
    RevColorSensorV3 colorSensor5;
    RevColorSensorV3 colorSensor6;

    BallState Astate = BallState.PURPLE;
    BallState Bstate = BallState.GREEN;
    BallState Cstate = BallState.EMPTY;

    //TODO: use enums to denote if a slot has a ball or not


    public void init() {
        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor1");
        colorSensor1.setGain(2);
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor2");
        colorSensor2.setGain(2);
        colorSensor3 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor3");
        colorSensor3.setGain(2);
        colorSensor4 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor4");
        colorSensor4.setGain(2);
        colorSensor5 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor5");
        colorSensor5.setGain(2);
        colorSensor6 = hardwareMap.get(RevColorSensorV3.class, "ColorSensor6");
        colorSensor6.setGain(2);

        sensorList.add(colorSensor1);
        sensorList.add(colorSensor2);
        sensorList.add(colorSensor3);
        sensorList.add(colorSensor4);
        sensorList.add(colorSensor5);
        sensorList.add(colorSensor6);

    }

    @Override
    public void loop() {
    }

    public boolean isGreen(RevColorSensorV3 sensor){
        NormalizedRGBA colors = sensor.getNormalizedColors();

        float r = colors.red;
       float g = colors.green;
       float b = colors.blue;

        int red = (int)(r * 255);
        int green = (int)(g * 255);
        int blue = (int)(b * 255);
        if (green >= 25 && blue <= 30 && red <= 10){
            return true;
        }

        return false;
    }
        public boolean isPurple(RevColorSensorV3 sensor){
            NormalizedRGBA colors = sensor.getNormalizedColors();

            float r = colors.red;
            float g = colors.green;
            float b = colors.blue;

            int red = (int)(r * 255);
            int green = (int)(g * 255);
            int blue = (int)(b * 255);
            if (blue >= 25 && red >= 10 && green <= 15){
                return true;
            }
            return false;
    }
    public boolean isEmpty(RevColorSensorV3 sensor){
        NormalizedRGBA colors = sensor.getNormalizedColors();

        float r = colors.red;
        float g = colors.green;
        float b = colors.blue;

        int red = (int)(r * 255);
        int green = (int)(g * 255);
        int blue = (int)(b * 255);
        if (blue <= 2 && red <= 2 && green <= 2){
            return true;
        }
        return false;
    }
    public int getNumBalls (){
        int num = 0;
        if (!isEmpty(colorSensor1) || !isEmpty(colorSensor2)){
            num++;
        }
        if (!isEmpty(colorSensor3) || !isEmpty(colorSensor4)) {
            num++;
        }
        if (!isEmpty(colorSensor5) || !isEmpty(colorSensor6)){
            num++;
        }

        return num;
    }
    public void shootGreen (){
        if (getNumBalls() == 0){
            return;
        }



    }
    public void updateStates (){
        if (!isEmpty(colorSensor1) || !isEmpty(colorSensor2)) {
            if (isGreen(colorSensor1) || (isGreen(colorSensor2))) {
                Astate = BallState.GREEN;
            } else {
                Astate = BallState.PURPLE;

            }
            if (!isEmpty(colorSensor3) || !isEmpty(colorSensor4)) {
                if (isGreen(colorSensor3) || (isGreen(colorSensor4))) {
                    Bstate = BallState.GREEN;
                } else {
                    Bstate = BallState.PURPLE;

                }
                if (!isEmpty(colorSensor5) || !isEmpty(colorSensor6)) {
                    if (isGreen(colorSensor5) || (isGreen(colorSensor6))) {
                        Cstate = BallState.GREEN;
                    } else {
                        Cstate = BallState.PURPLE;

                    }
                }

            }
        }
    }
}

