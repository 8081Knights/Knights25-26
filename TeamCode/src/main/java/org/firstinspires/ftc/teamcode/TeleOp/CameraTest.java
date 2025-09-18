package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="cameraTest")
public class CameraTest extends OpMode {
    //AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    List<AprilTagDetection> detections;
    final int TAGID = 20;
    final int frameWidth = 1280;
    int cameraBuffer = frameWidth / 3;


    VisionPortal.Builder vBuilder = new VisionPortal.Builder();

    HardwareSoftware robot = new HardwareSoftware();

    CRServo cameraServo;

    public void init() {
        // Create the AprilTag processor
        //tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Attach to the VisionPortal (this manages the camera stream)
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        cameraServo = hardwareMap.get(CRServo.class, "servoExample");

        telemetry.addLine("Initialized. Press Play.");
        telemetry.update();

    }

    @Override
    public void loop() {

        detections = aprilTag.getDetections();
        for (AprilTagDetection det : detections) {
            telemetry.addData("ID", det.id);
            telemetry.addData("Center", "(%.2f, %.2f)", det.center.x, det.center.y);
            telemetry.addData("Pose X", det.ftcPose.x);
            telemetry.addData("Pose Y", det.ftcPose.y);
            telemetry.addData("Heading (deg)", det.ftcPose.yaw);
            // cameraServo.setPower(.2);
//                    if (det.ftcPose.yaw < 0){
//                        cameraServo.setPower(tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.25);
//                    }
//                    if (det.ftcPose.yaw > 0){
//                        cameraServo.setPower(-.2);
//                    }
            //List<AprilTagDetection> result = tagProcessor.getDetections();
        }


        if (!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                //if (tag.id == TAGID) {
                telemetry.addData("TAG OUT", tag.center.x);
                                /*if (tag.center.x >= 0 && tag.center.x <= cameraBuffer) {
                                    cameraServo.setPower(-.5);
                                }
                                else if (tag.center.x >= (frameWidth - cameraBuffer) && tag.center.x <= frameWidth) {
                                    cameraServo.setPower(.5);
                                }
                                else {
                                    cameraServo.setPower(0);
                                }*/
                double num = -((tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.2);
                cameraServo.setPower(num);
                telemetry.addData("thingy", (num));
                //}
            }
        } else {
            telemetry.addData("TAG OUT", "NONE");
            cameraServo.setPower(0);
        }

        telemetry.update();

    }
}

