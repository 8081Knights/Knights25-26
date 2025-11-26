package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Represents the new position and rotation of the robot.
 */
public class NewPositionOfRobot {
	public boolean justDrive;
	public double newx, newy;
	public double newRotation;
	public double speed = .3;

	/**
	 * Sets the robot's future position and rotation.
	 *
	 * @param nx     The new x-coordinate.
	 * @param ny     The new y-coordinate.
	 * @param newRot The new rotation angle.
	 */
	public NewPositionOfRobot(double nx, double ny, double newRot) {
		this.newx = nx;
		this.newy = ny;
		this.newRotation = newRot;
		justDrive = true;
	}

	public NewPositionOfRobot(double nx, double ny, double newRot, double setspeed) {
		this.newx = nx;
		this.newy = ny;
		this.newRotation = newRot;
		this.speed = setspeed;
		justDrive = true;
	}

}