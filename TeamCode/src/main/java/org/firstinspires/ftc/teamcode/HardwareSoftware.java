
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareSoftware {

    private HardwareMap hw = null;


    //wheels
    public DcMotorEx FRdrive = null;
    public DcMotorEx BRdrive = null;
    public DcMotorEx BLdrive = null;
    public DcMotorEx FLdrive = null;

    public DcMotorEx flyWheel = null;

    public DcMotorEx intake = null;

    public Servo flyWheelRotator1 = null;
    public Servo flyWheelRotator2 = null;

    public Servo sorterServo = null;


    public SparkFunOTOS gyro;


    public void init(HardwareMap ahw) {

        hw = ahw;

        FLdrive = hw.get(DcMotorEx.class, "FLdrive");
        FRdrive = hw.get(DcMotorEx.class, "FRdrive");
        BLdrive = hw.get(DcMotorEx.class, "BLdrive");
        BRdrive = hw.get(DcMotorEx.class, "BRdrive");


        flyWheel = hw.get(DcMotorEx.class, "flyWheel");
        intake = hw.get(DcMotorEx.class, "intake");


        sorterServo = hw.get(Servo.class, "sorterServo");

        flyWheelRotator1 = hw.get(Servo.class, "flyWheel1");
        flyWheelRotator2 = hw.get(Servo.class, "flyWheel2");

        gyro = hw.get(SparkFunOTOS.class, "gyro");


        FLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flyWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        flyWheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         */

        //old robots set up
//        FLdrive.setDirection(DcMotorEx.Direction.REVERSE);
//        BLdrive.setDirection(DcMotorEx.Direction.FORWARD);
//        FRdrive.setDirection(DcMotorEx.Direction.REVERSE);
//        BRdrive.setDirection(DcMotorEx.Direction.FORWARD);

        flyWheelRotator2.setDirection(Servo.Direction.REVERSE);
        flyWheelRotator1.setDirection(Servo.Direction.FORWARD);


        //new robot setup
        FLdrive.setDirection(DcMotorEx.Direction.REVERSE);
        BLdrive.setDirection(DcMotorEx.Direction.REVERSE);
        FRdrive.setDirection(DcMotorEx.Direction.FORWARD);
        BRdrive.setDirection(DcMotorEx.Direction.FORWARD);

        FLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

}
