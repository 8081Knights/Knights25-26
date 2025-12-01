package org.firstinspires.ftc.teamcode;

public class HelperMethods {

	/**
	 * normalizes an angle - because it is cyclical, keeps it in range of (0, 2pi)
	 */
	public static double normalizeAngle(double angle) {
		while (angle < 0) {
			angle += 2 * Math.PI;
		}
		while (angle >= 2 * Math.PI) {
			angle -= 2 * Math.PI;
		}
		return angle;
	}


	/**
	 * rotates coordinates by theta.
	 * if you imagine a circle around 0, 0,
	 * and these cordinates on that circle, it will rotate them by theta degrees
	 *
	 * @param x
	 * @param y
	 * @param theta     degrees
	 * @param isDegrees
	 * @return double array with {x, y}
	 */
	public static double[] rotateCord(double x, double y, double theta, boolean isDegrees) {
		double[] cords = {0, 0};
		if (isDegrees) {
			theta = Math.toRadians(theta);
		}
		cords[0] = x * Math.cos(theta) - y * Math.sin(theta);
		cords[1] = x * Math.sin(theta) - y * Math.cos(theta);
		return cords;
	}
}
