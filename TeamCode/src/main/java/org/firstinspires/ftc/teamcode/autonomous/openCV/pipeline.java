package org.firstinspires.ftc.teamcode.autonomous.openCV;


import org.firstinspires.ftc.teamcode.autonomous.config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
	private final config _config = new config();
	private final Boolean isRed;
	Mat mat = new Mat();
	Mat lowMask = new Mat();
	Mat highMask = new Mat();
	Mat full = new Mat();

	private final Scalar redLow1 = _config.RED_LOW_HSV_LOWER_BOUNDARY;
	private final Scalar redLow2 = _config.RED_LOW_HSV_HIGHER_BOUNDARY;
	private final Scalar redHigh1 = _config.RED_HIGH_HSV_LOWER_BOUNDARY;
	private final Scalar redHigh2 = _config.RED_HIGH_HSV_HIGHER_BOUNDARY;

	private final Scalar blueLow = _config.BLUE_HSV_LOWER_BOUNDARY;
	private final Scalar blueHigh = _config.BLUE_HSV_HIGH_BOUNDARY;

	public pipeline(final Boolean _isRed) {
		isRed = _isRed;
	}

	private final Rect CENTER_RECT = new Rect(
		new Point(130, 80),
		new Point(210, 160)
	);
	private final Rect RIGHT_RECT = new Rect(
		new Point(),
		new Point()
	);
	private final Scalar COLOR = new Scalar(255, 0, 0);

	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

		if (isRed) redDetector();
		else blueDetector();

		Imgproc.rectangle(mat, CENTER_RECT, COLOR);
		Imgproc.rectangle(mat, RIGHT_RECT, COLOR);

		Imgproc.cvtColor(full, mat, Imgproc.COLOR_GRAY2RGB);
		return mat;
	}

	private void redDetector() {
		Core.inRange(mat, redLow1, redHigh1, lowMask);
		Core.inRange(mat, redLow2, redHigh2, highMask);
		Core.add(lowMask, highMask, full);

		lowMask.release();
		highMask.release();
		full.release();
	}

	private void blueDetector() {
		Core.inRange(mat, blueLow, blueHigh, full);

		full.release();
	}
}