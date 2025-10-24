/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import android.graphics.Color;
import android.util.Size;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="TurnAndGoToBall", group="Robot")

public class TurnAndGoToBall extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         LDrive   = null;
    private DcMotor         RDrive  = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    // final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
  //  static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = 33.3;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     inchesPerDegree = 0.125;
    Circle circleFit = null;

    ColorBlobLocatorProcessor.Blob currentBlob = null;

    List<ColorBlobLocatorProcessor.Blob> blobs = null;

    ColorBlobLocatorProcessor colorLocatorPurple = null;

    ColorBlobLocatorProcessor colorLocatorGreen = null;


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        LDrive  = hardwareMap.get(DcMotor.class, "FLdrive");
        RDrive = hardwareMap.get(DcMotor.class, "FRdrive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        LDrive.setDirection(DcMotor.Direction.REVERSE);
        RDrive.setDirection(DcMotor.Direction.FORWARD);

        LDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          LDrive.getCurrentPosition(),
                          RDrive.getCurrentPosition());
        telemetry.update();





        // Wait for the game to start (driver presses START)
        waitForStart();


        //Visual code!!!
        colorLocatorPurple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocatorPurple)
                .addProcessor(colorLocatorGreen)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);


        while (opModeIsActive() || opModeInInit()) {

            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list

            updateBlobs();


            telemetry.addLine("Circularity Radius Center");

            double ARTIFACT_REAL_WIDTH_CM = 12.7;
            double FOCAL_LENGTH_IN_PIXELS = 424.4;

            // Display the Blob's circularity, and the size (radius) and center location of its circleFit.
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                currentBlob = b;
                circleFit = currentBlob.getCircle();
                telemetry.addLine("HELLLLLLLLLLOOOOOOO "+ String.format("%5.3f      %3d     (%3d,%3d)",
                        b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
                telemetry.update();
//tyler is the best

                if(!blobs.isEmpty()){
                    // Assuming you care about the largest blob
                    ColorBlobLocatorProcessor.Blob largestBlob = blobs.get(0);

                    // Get the width of the bounding box around the blob in pixels
                    double pixelWidth = 2 * circleFit.getRadius();

                    // Calculate the distance
                    double distance = 0.9 * (ARTIFACT_REAL_WIDTH_CM * FOCAL_LENGTH_IN_PIXELS) / pixelWidth;

                    // the multiplier 0.9 is used to calibrate the distance. could use some fine tuning
                    // Send the information to the Driver Station
                    telemetry.addData("Distance to Artifact", "%.2f cm", distance);
                    rotateToBall();
                    encoderDrive(DRIVE_SPEED,  distance,  distance, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                    //encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                    // encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
                    telemetry.addLine("HELLLLLLLLLLOOOOOOO "+String.format("%5.3f      %3d     (%3d,%3d)",
                            b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
                    telemetry.addData("Path", "Complete");
                    telemetry.update();
                    sleep(1000);  // pause to display final telemetry message.
                }
            }


        }



            // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

    }

    public void updateBlobs(){
            blobs = colorLocatorPurple.getBlobs();


            blobs.addAll(colorLocatorGreen.getBlobs());



            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, blobs);  // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, blobs);

        if(!blobs.isEmpty()) {
            currentBlob = blobs.get(0);
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LDrive.setTargetPosition(newLeftTarget);
            RDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LDrive.setPower(Math.abs(speed));
            RDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                    (LDrive.isBusy() && RDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            LDrive.getCurrentPosition(), RDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LDrive.setPower(0);
            RDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void continuousEncoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, int X) {
        int newLeftTarget;
        int newRightTarget;
        int x = X;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = RDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            LDrive.setTargetPosition(newLeftTarget);
            RDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LDrive.setPower(Math.abs(speed));
            RDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LDrive.isBusy() && RDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        LDrive.getCurrentPosition(), RDrive.getCurrentPosition());
                telemetry.addData("X: ", x);
                updateBlobs();
                circleFit = currentBlob.getCircle();
                x = (int)circleFit.getX();
                telemetry.update();
            }
/*
            // Stop all motion;
            LDrive.setPower(0);
            RDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            LDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
            */

        }
    }

    public void turn(int degrees, double speed, int X){

        double inches = degrees * inchesPerDegree;
        inches = Math.abs(inches);
        if(degrees > 0){
            continuousEncoderDrive(speed, -inches ,inches, 10, X);
        } else {
            continuousEncoderDrive(speed, inches , -inches, 10, X);
        }
    }

    public void rotateToBall(){
        updateBlobs();
        circleFit = currentBlob.getCircle();
        int x = (int)circleFit.getX();
        telemetry.addData("X: ",x);

        while(x > 100){
            turn(-5, TURN_SPEED, x);
            updateBlobs();
            circleFit = currentBlob.getCircle();
            x = (int)circleFit.getX();
            telemetry.update();
        }
        while(x < 50){
            turn(5, TURN_SPEED, x);
            updateBlobs();
            circleFit = currentBlob.getCircle();
            x = (int)circleFit.getX();
            telemetry.update();
        }
    }
}
