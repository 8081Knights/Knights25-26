package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;

@TeleOp(name = "Hood Test")
public class HoodedTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    double pos = 0.5;
    int greenPosMotif = 0;
    int currentServo = 1;

    ArrayList<Integer> nums = new ArrayList<>();

    ArrayList<Servo> servos = new ArrayList<>();
    ArrayList<Servo> restServos = new ArrayList<>();
    ArrayList<Servo> activeServos = new ArrayList<>();

    double activeServoPos = 0.7;

    double restServoPos = 0.3;

    boolean isSorting = false;

    int numServosMoved = 0;

	@Override
	public void init() {

        servos.add(robot.sorter1);
        servos.add(robot.sorter2);
        servos.add(robot.sorter3);
        restServos.clear();
        restServos.addAll(servos);
    }

	@Override
	public void loop() {

		if (gamepad1.x) {
			robot.turnTableRotator.setPower(0.9);
		} else if (gamepad1.y) {
			robot.turnTableRotator.setPower(-0.9);
		} else {
			robot.turnTableRotator.setPower(0);
		}

        telemetry.addData("turntablething", robot.turnTableRotator.getCurrentPosition());

		if (gamepad1.right_trigger > 0.5) {
        //robot.flyWheel.setPower(0.95);
        robot.flyWheel.setVelocity(2100);
		} else if (gamepad1.left_trigger > 0.5){
            robot.flyWheel.setVelocity(1500);
        } else {
			robot.flyWheel.setPower(0);
		}
        telemetry.addData("Flywheel velo", robot.flyWheel.getVelocity());
        telemetry.addData("flywheel power", robot.flyWheel.getPower());

        //rand number between 1 and 3
        if(gamepad1.dpad_up) {
            currentServo = (int) (Math.random() * 3) + 1;
        }

        if(gamepad1.dpad_down){
            isSorting = true;
            restServos.remove(servos.get(currentServo - 1));
            activeServos.add(servos.get(currentServo - 1));
        }

        if(isSorting){

            if(Math.abs(servos.get(currentServo - 1).getPosition() - activeServoPos) < 0.002){
                currentServo++;
                numServosMoved++;
                if(currentServo > 3){
                    currentServo = 1;
                }
                restServos.remove(servos.get(currentServo - 1));
                activeServos.add(servos.get(currentServo - 1));

                if(numServosMoved > 2){
                    numServosMoved = 0;
                    isSorting = false;
                    activeServos.clear();
                    restServos.add(servos.get(0));
                    restServos.add(servos.get(1));
                    restServos.add(servos.get(2));
                }

            }
        }

        for (Servo s : restServos) {
          s.setPosition(restServoPos);
        }

        Servo s = servos.get(currentServo - 1);
        double current = s.getPosition();
        double target = activeServoPos;
        double step = 0.003;   // smaller = slower
        if (Math.abs(current - target) > 0.002) {
            if (current < target) {
                s.setPosition(Math.min(current + step, target));
            } else {
                s.setPosition(Math.max(current - step, target));
            }
        }

        telemetry.addData("currentServo", currentServo);
        if(greenPosMotif != 0) {
            telemetry.addData("greenPos", greenPosMotif);
        }

        telemetry.addData("sort1", robot.sorter1.getPosition());
        telemetry.addData("sort2", robot.sorter2.getPosition());
        telemetry.addData("sort3", robot.sorter3.getPosition());



        if (gamepad1.a) {
            pos += 0.02;
        }
		 else if (gamepad1.b) {
            pos -= 0.02;
         }
         pos = Math.min(1, pos);
         robot.flyWheelRotator.setPosition(pos);
         telemetry.addData("hood pos", pos);

	}
}
