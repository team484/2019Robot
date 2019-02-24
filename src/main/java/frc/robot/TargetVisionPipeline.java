/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.vision.VisionPipeline;

/**
 * Add your docs here.
 */
public class TargetVisionPipeline implements VisionPipeline {
    private Mat hsvThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> convexHullsOutput = new ArrayList<MatOfPoint>();
	private ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<MatOfPoint>();

	//Constants
	private static final double[] hsvThresholdHue = {60.0, 75.0};
	private static final double[] hsvThresholdSaturation = {120.0, 255.0};
	private static final double[] hsvThresholdValue = {100.0, 255.0};
	private static final double filterContoursMinArea = 50.0;
	private static final double filterContoursMinWidth = 4.0;
	private static final double filterContoursMinHeight = 5.0;
	
	static {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
	}

	public Mat hsvThresholdInput;
	/**
	 * This is the primary method that runs the pipeline and updates the outputs.
	 */
    @Override
    public void process(Mat source0) {
		if (Robot.disableVision) {
			return;
		}
		// Step HSV_Threshold:
		hsvThresholdInput = source0;

		hsvThreshold(hsvThresholdInput, hsvThresholdHue, hsvThresholdSaturation, hsvThresholdValue, hsvThresholdOutput);

		// Step Find_Contours:
		Mat findContoursInput = hsvThresholdOutput;
		findContours(findContoursInput, true, findContoursOutput);

		// Step Convex_Hulls:
		ArrayList<MatOfPoint> convexHullsContours = findContoursOutput;
		convexHulls(convexHullsContours, convexHullsOutput);

		// Step Filter_Contours:
		ArrayList<MatOfPoint> filterContoursContours = convexHullsOutput;
		filterContours(filterContoursContours, filterContoursMinArea, filterContoursMinWidth, filterContoursMinHeight, filterContoursOutput);

	}

	/**
	 * This method is a generated getter for the output of a HSV_Threshold.
	 * @return Mat output from HSV_Threshold.
	 */
	public Mat hsvThresholdOutput() {
		return hsvThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}

	/**
	 * This method is a generated getter for the output of a Convex_Hulls.
	 * @return ArrayList<MatOfPoint> output from Convex_Hulls.
	 */
	public ArrayList<MatOfPoint> convexHullsOutput() {
		return convexHullsOutput;
	}

	/**
	 * This method is a generated getter for the output of a Filter_Contours.
	 * @return ArrayList<MatOfPoint> output from Filter_Contours.
	 */
	public ArrayList<MatOfPoint> filterContoursOutput() {
		return filterContoursOutput;
	}

	public Mat startImage() {
		return hsvThresholdInput;
	}

	/**
	 * Segment an image based on hue, saturation, and value ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param val The min and max value
	 * @param output The image in which to store the output.
	 */
	private static void hsvThreshold(Mat input, double[] hue, double[] sat, double[] val,
	    Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HSV);
		Core.inRange(out, new Scalar(hue[0], sat[0], val[0]),
			new Scalar(hue[1], sat[1], val[1]), out);
	}

	/**
	 * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
	 * @param input The image on which to find the contours of.
	 * @param externalOnly If only external contours should be found.
	 * @param contours The output contours list
	 */
	private static void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}

	/**
	 * Compute the convex hulls of contours.
	 * @param inputContours The contours on which to perform the operation.
	 * @param outputContours The contours where the output will be stored.
	 */
	private static void convexHulls(List<MatOfPoint> inputContours,
		ArrayList<MatOfPoint> outputContours) {
		final MatOfInt hull = new MatOfInt();
		outputContours.clear();
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final MatOfPoint mopHull = new MatOfPoint();
			Imgproc.convexHull(contour, hull);
			mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
			for (int j = 0; j < hull.size().height; j++) {
				int index = (int) hull.get(j, 0)[0];
				double[] point = new double[] {contour.get(index, 0)[0], contour.get(index, 0)[1]};
				mopHull.put(j, 0, point);
			}
			outputContours.add(mopHull);
		}
	}


	/**
	 * Filters out contours that do not meet certain criteria.
	 * @param inputContours is the input list of contours
	 * @param output is the the output list of contours
	 * @param minArea is the minimum area of a contour that will be kept
	 * @param minWidth minimum width of a contour
	 * @param minHeight minimum height
	 * @param minRatio minimum ratio of width to height
	 * @param maxRatio maximum ratio of width to height
	 */
	private static void filterContours(List<MatOfPoint> inputContours, double minArea,
		double minWidth, double minHeight,
		List<MatOfPoint> output) {
		output.clear();
		//operation
		for (int i = 0; i < inputContours.size(); i++) {
			final MatOfPoint contour = inputContours.get(i);
			final Rect bb = Imgproc.boundingRect(contour);
			if (bb.width < minWidth) continue;
			if (bb.height < minHeight) continue;
			final double area = Imgproc.contourArea(contour);
			if (area < minArea) continue;
			output.add(contour);
		}
	}
}
