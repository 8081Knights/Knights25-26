
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HardwareSoftware {

    private HardwareMap hw = null;


    //wheels
    public DcMotorEx FRdrive = null;
    public DcMotorEx BRdrive = null;
    public DcMotorEx BLdrive = null;
    public DcMotorEx FLdrive = null;

    public DcMotorEx flyWheel = null;

    public DcMotorEx intake = null;

    public Servo flyWheelRotator = null;
    public DcMotorEx turnTableRotator = null;

    public Servo sorter1 = null;
    public Servo sorter2 = null;
    public Servo sorter3 = null;



    public SparkFunOTOS gyro;




    /**
     * initializes the motors and servos
     * @param ahw
     */
    public void init(HardwareMap ahw) {

        hw = ahw;

        FLdrive = hw.get(DcMotorEx.class, "FLdrive");
        FRdrive = hw.get(DcMotorEx.class, "FRdrive");
        BLdrive = hw.get(DcMotorEx.class, "BLdrive");
        BRdrive = hw.get(DcMotorEx.class, "BRdrive");


        flyWheelRotator = hw.get(Servo.class, "flyWheelRotator");
        turnTableRotator = hw.get(DcMotorEx.class, "turnTableRotator");
        flyWheel = hw.get(DcMotorEx.class, "flyWheel");
        sorter1 = hw.get(Servo.class, "sorter1");
        sorter2 = hw.get(Servo.class, "sorter2");
        sorter3 = hw.get(Servo.class, "sorter3");


        intake = hw.get(DcMotorEx.class, "intake");



        gyro = hw.get(SparkFunOTOS.class, "gyro");


        FLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flyWheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);


        FLdrive.setDirection(DcMotorEx.Direction.FORWARD);
        BLdrive.setDirection(DcMotorEx.Direction.REVERSE);
        FRdrive.setDirection(DcMotorEx.Direction.FORWARD);
        BRdrive.setDirection(DcMotorEx.Direction.FORWARD);

        FLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


    }

    /**
     * initialize the gyro stuff
     */
    public void initGyro() {
        gyro.calibrateImu();
        gyro.resetTracking();

        gyro.setLinearUnit(DistanceUnit.INCH);
        gyro.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-6.186, 0.7, 0);
        gyro.setOffset(offset);
        gyro.setLinearScalar(1.0);
        gyro.setAngularScalar(1.0);
        gyro.calibrateImu();
        gyro.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        gyro.setPosition(currentPosition);
    }

    /**
     * initialize the gyro and add current spot on field to make the positions always the same
     * @param x
     * @param y
     * @param h
     */
    public void initGyro(double x, double y, double h) {
        gyro.calibrateImu();
        gyro.resetTracking();

        gyro.setLinearUnit(DistanceUnit.INCH);
        gyro.setAngularUnit(AngleUnit.RADIANS);

        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-6.186, 0.7, 0);
        gyro.setOffset(offset);
        gyro.setLinearScalar(1.0);
        gyro.setAngularScalar(1.0);
        gyro.calibrateImu();
        gyro.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, h);
        gyro.setPosition(currentPosition);
    }

}
