package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;
@TeleOp(name = "Servo Test AI")
public class ServoTestAI extends OpMode {

    double pos = 0.5;
    int greenPosMotif = 0;
    int currentServo = 1;

    ArrayList<Integer> nums = new ArrayList<>();

    ArrayList<Servo> servos = new ArrayList<>();
    ArrayList<Servo> activeServos = new ArrayList<>();

    double activeServoPos = 0.3;

    double restServoPos = 0;

    boolean isSorting = false;

    int velocity = -1000;

    int numServosMoved = 0;

    Servo sorterA;
    Servo sorterB;
    Servo sorterC;
    DcMotorEx flyWheel;
    DcMotorEx turnTableRotator;

    Servo flyWheelAdjuster = null;

    double hoodPos = 0.5;

    final double UP_POS = 0.7;
    final double DOWN_POS = 0.0;

    final double UP_TIME = 1.00;    // time to flick

    enum FeedState {
        IDLE,
        SERVO_UP,
        SERVO_DOWN
    }

    FeedState feedState = FeedState.IDLE;
    int servoIndex = 0;
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        sorterA = hardwareMap.get(Servo.class, "sorter1");
        sorterB = hardwareMap.get(Servo.class, "sorter2");
        sorterC = hardwareMap.get(Servo.class, "sorter3");


        //flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        //turnTableRotator = hardwareMap.get(DcMotorEx.class, "turntable");
        //flyWheelAdjuster = hardwareMap.get(Servo.class, "flyWheelAdjuster");


        servos.add(sorterA);
        servos.add(sorterB);
        servos.add(sorterC);
    }

    @Override
    public void loop() {
//		if (gamepad1.x) {
//			turnTableRotator.setPower(0.9);
//		} else if (gamepad1.y) {
//			turnTableRotator.setPower(-0.9);
//		} else {
//			turnTableRotator.setPower(0);
//		}


        //telemetry.addData("turntablething", turnTableRotator.getCurrentPosition());
        //1150 works for shooting from close
        //700 is waay to slow
        //if (gamepad1.right_trigger > 0.5) {
            //robot.flyWheel.setPower(0.95);
            //flyWheel.setVelocity(1800);
        //} else if (gamepad1.left_trigger > 0.5) {
            // flyWheel.setVelocity(1150);
        //} else if (gamepad1.left_bumper) {
            //flyWheel.setVelocity(velocity);
        //} else {
            // flyWheel.setPower(0);
        //}

//        if (gamepad1.left_stick_y > 0.1) {
//            velocity += 4;
//        } else if (gamepad1.left_stick_y < -0.1) {
//            velocity -= 4;
//        }
        // telemetry.addData("flywheel custom velo: ", velocity);
        // telemetry.addData("Flywheel velo", flyWheel.getVelocity());
        // telemetry.addData("flywheel power", flyWheel.getPower());

//        if (gamepad1.dpad_left) {
//            hoodPos += 0.0001;
//        } else if (gamepad1.dpad_right) {
//            hoodPos -= 0.0001;
//        }
//        if (gamepad1.b) {
//            //  flyWheelAdjuster.setPosition(hoodPos);
//        }

        //telemetry.addData("adjusterPos", hoodPos);
        //get position method is basically a get target position method, does not actually read the current position

        //rand number between 1 and 3


        if((gamepad1.a || gamepad1.b || gamepad1.x)){
            if(feedState == FeedState.IDLE){
                servoIndex = 0;
                feedState = FeedState.SERVO_UP;
                timer.reset();
            }


            if(gamepad1.a && !activeServos.contains(sorterA)){
                activeServos.add(sorterA);
            }
            if(gamepad1.b && !activeServos.contains(sorterB)){
                activeServos.add(sorterB);
            }
            if(gamepad1.x && !activeServos.contains(sorterC)){
                activeServos.add(sorterC);
            }

        }

        if (gamepad1.dpad_down && feedState == FeedState.IDLE) {
            servoIndex = 0;
            feedState = FeedState.SERVO_UP;
            timer.reset();
        }

        switch (feedState) {

            case SERVO_UP:
                activeServos.get(servoIndex).setPosition(UP_POS);
                if (timer.seconds() > UP_TIME) {
                    feedState = FeedState.SERVO_DOWN;
                    timer.reset();
                }
                break;

            case SERVO_DOWN:
                activeServos.get(servoIndex).setPosition(DOWN_POS);
                if (timer.seconds() > 0.12) {
                    servoIndex++;
                    if (servoIndex >= activeServos.size()) {
                        feedState = FeedState.IDLE;
                        activeServos.clear();
                    } else {
                        feedState = FeedState.SERVO_UP;
                    }
                    timer.reset();
                }
                break;

            case IDLE:
                for (Servo s : servos) {
                    s.setPosition(DOWN_POS);
                }
                break;
        }

        telemetry.addData("currentServo", currentServo);
        if(greenPosMotif != 0) {
            telemetry.addData("greenPos", greenPosMotif);
        }

        telemetry.addData("sort1", sorterA.getPosition());
        telemetry.addData("sort2", sorterB.getPosition());
        telemetry.addData("sort3", sorterC.getPosition());
//
//
//
//        if (gamepad1.a) {
//            pos += 0.02;
//        }
//		 else if (gamepad1.b) {
//            pos -= 0.02;
//         }
//         pos = Math.min(1, pos);
//         //flyWheelRotator.setPosition(pos);
//         telemetry.addData("hood pos", pos);

    }
}
