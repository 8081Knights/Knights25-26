package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "servo quick")
public class ServoTestQuick extends OpMode {
    Servo servo = null;
    DcMotorEx flyWheel = null;
    double highPos = 0.91;
    double lowPos = 0.52;
    double currentPos = lowPos;

    private ElapsedTime rampTime = new ElapsedTime();
    private ElapsedTime servoTime = new ElapsedTime();

    boolean isRamping = false;

    int flyWheelVelo = 0;

    int maxFlyWheelVelo = 1600;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "sorter1");
        //flyWheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        servoTime.reset();
    }

    @Override
    public void loop() {
        if(gamepad1.left_stick_y > 0){
            highPos += 0.0001;
        } else if(gamepad1.left_stick_y < 0){
            highPos -= 0.0001;
        }

        if(gamepad1.right_stick_y > 0){
            lowPos += 0.0001;
        } else if(gamepad1.right_stick_y < 0){
            lowPos -= 0.0001;
        }

//        if(gamepad1.x && !isRamping){
//            rampTime.reset();
//            isRamping = true;
//        }

        if(isRamping){
            //100 is the degrees per second it accelerates
            flyWheelVelo = (int)(rampTime.seconds() * 100.0);
            if(flyWheelVelo > maxFlyWheelVelo){
                flyWheelVelo = maxFlyWheelVelo;
            }
        }

        //flyWheel.setVelocity(-flyWheelVelo);
        if(isRamping) {
            telemetry.addData("rampTime", rampTime.seconds());
        }

       // telemetry.addData("flywheel velo", flyWheel.getVelocity());

        telemetry.addData("y", gamepad1.left_stick_y);

        telemetry.addData("servoTime", servoTime.seconds());


        if(gamepad1.a  && servoTime.seconds()  > 0.5){
            servoTime.reset();
            if(currentPos == lowPos){
                currentPos = highPos;
            } else {
                currentPos = lowPos;
            }
        }
        servo.setPosition(currentPos);
        telemetry.addData("servoPos", currentPos);
        telemetry.addData("high Pos:", highPos);
        telemetry.addData("low pos", lowPos);

    }
}
