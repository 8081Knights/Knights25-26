package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "dual servo test")
public class FlipperTest extends OpMode {
    Servo servo1 = null;
    Servo servo2 = null;

    DcMotorEx flywheel = null;

    double s2high = 0.95;
    double s2low = 0.63;

    double s1high = 0.63;
    double s1low = 0.31;

    private ElapsedTime sort1Time = new ElapsedTime();
    private ElapsedTime sort2Time = new ElapsedTime();

    private ElapsedTime rampTime = new ElapsedTime();

    boolean isRamping = false;

    int flyWheelVelo = 0;

    final int maxFlyWheelVelo = 1600;
    private final double targetRampTime = 5;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "sorter1");
        servo2 = hardwareMap.get(Servo.class, "sorter2");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        sort1Time.reset();
        sort2Time.reset();
    }

    @Override
    public void loop() {
        if(gamepad1.a && sort1Time.seconds() > 0.5){
            sort1Time.reset();
            if(Math.abs(servo1.getPosition() - s1high) < 0.1){
                servo1.setPosition(s1low);
            } else{
                servo1.setPosition(s1high);
            }
        }

        if(gamepad1.b && sort2Time.seconds() > 0.5){
            sort2Time.reset();
            if(Math.abs(servo2.getPosition() - s2high) < 0.1){
                servo2.setPosition(s2low);
            } else{
                servo2.setPosition(s2high);
            }
        }



        if(gamepad1.x && !isRamping){
            rampTime.reset();
            isRamping = true;
        }

        if(isRamping){
            //100 is the degrees per second it accelerates
            flyWheelVelo = (int)(getVelo(rampTime.seconds()));
            if(flyWheelVelo > maxFlyWheelVelo){
                flyWheelVelo = maxFlyWheelVelo;
            }
        }

        flywheel.setVelocity(-flyWheelVelo);

        if(isRamping) {
            telemetry.addData("rampTime", rampTime.seconds());
        }

        telemetry.addData("flywheel velo", flywheel.getVelocity());

        telemetry.addData("servo2Pos", servo2.getPosition());
        telemetry.addData("servo1Pos", servo1.getPosition());

        telemetry.addData("times", sort1Time + " " + sort2Time);

    }


    public double getVelo(double seconds){
        return seconds * (maxFlyWheelVelo / targetRampTime);
    }
}
