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
	Mat mat = new Mat();
	Mat lowMask = new Mat();
	Mat highMask = new Mat();
	Mat full = new Mat();

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

		redDetector();

		Imgproc.rectangle(mat, CENTER_RECT, COLOR);
		Imgproc.rectangle(mat, RIGHT_RECT, COLOR);
		return mat;
	}

	private void redDetector() {
		Scalar low1 = _config.RED_LOW_HSV_LOWER_BOUNDARY;
		Scalar low2 = _config.RED_LOW_HSV_HIGHER_BOUNDARY;
		Scalar high1 = _config.RED_HIGH_HSV_LOWER_BOUNDARY;
		Scalar high2 = _config.RED_HIGH_HSV_HIGHER_BOUNDARY;

		Core.inRange(mat, low1, high1, lowMask);
		Core.inRange(mat, low2, high2, highMask);
		Core.add(lowMask, highMask, full);

		Imgproc.cvtColor(full, mat, Imgproc.COLOR_GRAY2RGB);

		lowMask.release();
		highMask.release();
		full.release();
	}
}