package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

public class CameraSensor {
	//class to handle all camera detection logic,
	// uses logitech camera

	private ColorBlobLocatorProcessor colorLocatorPurple = null;

	private ColorBlobLocatorProcessor colorLocatorGreen = null;

	private AprilTagProcessor aprilTag;
	static ArrayList<VisionProcessor> processors = new ArrayList<>();

	static List<ColorBlobLocatorProcessor.Blob> blobs = null;

	static Circle circleFit = null;
	static ColorBlobLocatorProcessor.Blob currentBlob = null;

	static HardwareSoftware robot;


	public ArrayList<AprilTagDetection> getTagDetections() {
		return aprilTag.getDetections();
	}

	public VisionPortal initVision() {
		processors = new ArrayList<>();
		aprilTag = null;
		colorLocatorGreen = null;
		colorLocatorPurple = null;


		aprilTag = initTagProcessor();
		colorLocatorGreen = initGreenLocator();
		colorLocatorPurple = initPurpleLocator();

		processors.add(aprilTag);
		processors.add(colorLocatorPurple);
		processors.add(colorLocatorGreen);
		return initVisionProcessor(processors);
	}

	public static ColorBlobLocatorProcessor initGreenLocator() {
		return new ColorBlobLocatorProcessor.Builder()
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
	}

	public static ColorBlobLocatorProcessor initPurpleLocator() {
		return new ColorBlobLocatorProcessor.Builder()
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
	}

	public static VisionPortal initVisionProcessor(ArrayList<VisionProcessor> list) {
		WebcamName cam = hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1");
		VisionPortal.Builder o = new VisionPortal.Builder();
		o.setCamera(cam);

		for (VisionProcessor processor : list) {
			o.addProcessor(processor);
		}


		return o.build();
	}

	public static AprilTagProcessor initTagProcessor() {
		return new AprilTagProcessor.Builder()
				.setDrawAxes(true)
				.setDrawCubeProjection(true)
				.setDrawTagOutline(true)
				.build();
	}

	/**
	 * if you want to use color blobs,
	 * the fields from the currentBlob and circle fit,
	 * also the distance from blob variable too
	 */
	public double handleBlobs() {
		updateBlobs();
		return getClosestDistance();
	}


	public static double getClosestDistance() {
		double distanceFromBlob = 0;
		double ARTIFACT_REAL_WIDTH_CM = 12.7;
		double FOCAL_LENGTH_IN_PIXELS = 424.4;

// Display the Blob's circularity, and the size (radius) and center location of its circleFit.
		//for (ColorBlobLocatorProcessor.Blob b : blobs) {
		//	currentBlob = b;
		//	circleFit = currentBlob.getCircle();
		//telemetry.addLine("HELLLLLLLLLLOOOOOOO " + String.format("%5.3f      %3d     (%3d,%3d)",
		//currentBlob.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));

		if (!blobs.isEmpty()) {
			// Assuming you care about the largest blob
			ColorBlobLocatorProcessor.Blob largestBlob = blobs.get(0);
			circleFit = largestBlob.getCircle();

			// Get the width of the bounding box around the blob in pixels
			double pixelWidth = 2 * circleFit.getRadius();

			// Calculate the distance
			distanceFromBlob = 0.9 * (ARTIFACT_REAL_WIDTH_CM * FOCAL_LENGTH_IN_PIXELS) / pixelWidth;

			// the multiplier 0.9 is used to calibrate the distance. could use some fine tuning
			// Send the information to the Driver Station
			//telemetry.addData("Distance to Artifact", "%.2f cm", distanceFromBlob);

			//telemetry.addLine("HELLLLLLLLLLOOOOOOO " + String.format("%5.3f      %3d     (%3d,%3d)",
			//       currentBlob.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
			//telemetry.addData("Path", "Complete");
			//	}
		}
		return distanceFromBlob;
	}

	public List<ColorBlobLocatorProcessor.Blob> getColorBlobs() {
		updateBlobs();
		return blobs;
	}


	public void updateBlobs() {
		blobs = colorLocatorPurple.getBlobs();

		blobs.addAll(colorLocatorGreen.getBlobs());

		ColorBlobLocatorProcessor.Util.filterByCriteria(
				ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
				50, 20000, blobs);  // filter out very small blobs.

		ColorBlobLocatorProcessor.Util.filterByCriteria(
				ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blobs);

	}
}
