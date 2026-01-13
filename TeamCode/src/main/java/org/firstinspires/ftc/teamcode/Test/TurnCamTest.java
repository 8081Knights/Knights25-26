package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;



import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp(name = "turnCamTest")
public class TurnCamTest extends OpMode {
    DcMotorEx turntable;

    VisionPortal visionPortal;

    AprilTagProcessor aprilTag;

    List<AprilTagDetection> detections;

    AprilTagDetection bestDet = null;
    double bearing = 0;

    double ticksPerDegree = 4.71472;


    @Override
    public void init() {
        turntable = hardwareMap.get(DcMotorEx.class, "turntable");
        turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

    }

    @Override
    public void loop() {

        detections = aprilTag.getDetections();
        if(detections.isEmpty()){
            bestDet = null;
        } else {
            for (AprilTagDetection det : detections) {
                //if (det.id == 24 || det.id == 20) {
                bestDet = det;
                //}
            }
        }

        if(bestDet != null){
            bearing = bestDet.ftcPose.bearing;
        } else {
            bearing = 0;
        }

        telemetry.addData("bearing", bearing);
        telemetry.addData("tag", bestDet);

        double kP = 0.006; // tune

        double turnPower = kP * bearing;
        turnPower = Range.clip(turnPower, -0.3, 0.3);
        if(gamepad1.a){
            turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turntable.setPower(turnPower);
        } else {
            turntable.setPower(0);
        }


//        if(gamepad1.a){
//            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turntable.setTargetPosition(degreesToTicks(360));
//            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turntable.setPower(0.6);
//        }else if (gamepad1.x){
//            turntable.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            turntable.setTargetPosition(degreesToTicks(-360));
//            turntable.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            turntable.setPower(0.6);
//        }

//        if(gamepad1.right_trigger > 0.5){
//            turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            turntable.setVelocity(400);
//        } else if (gamepad1.left_trigger > 0.5){
//            turntable.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            turntable.setVelocity(-400);
//        } else if(turntable.getMode() == DcMotor.RunMode.RUN_USING_ENCODER){
//            turntable.setVelocity(0);
//        }

        telemetry.addData("velocity", turntable.getVelocity());
        telemetry.addData("pos", turntable.getCurrentPosition());


    }

    public int degreesToTicks(int degrees){
        return (int)(degrees * ticksPerDegree);
    }
    /*
    public void turnToTag(){
        while (bearing > 100) {
            turn(-5, TURN_SPEED, x);
            telemetry.update();
        }
        while (bearing < 50) {
            turn(5, TURN_SPEED, x);
            telemetry.update();
        }
    }



    public void turn(int degrees, double speed, int X) {

        double inches = degrees * inchesPerDegree;
        inches = Math.abs(inches);
        if (degrees > 0) {
            continuousEncoderDrive(speed, -inches, inches, 10, X);
        } else {
            continuousEncoderDrive(speed, inches, -inches, 10, X);
        }
    }

     */
}
