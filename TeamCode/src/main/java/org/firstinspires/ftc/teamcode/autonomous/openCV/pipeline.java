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

	private final Mat mat = new Mat();
	private final Mat lowMask = new Mat();
	private final Mat highMask = new Mat();
	private final Mat full = new Mat();

	private final Scalar RED_LOW_1 = _config.RED_HSV_LOW1_BOUNDARY;
	private final Scalar RED_HIGH_1 = _config.RED_HSV_HIGH1_BOUNDARY;
	private final Scalar RED_LOW_2 = _config.RED_HSV_LOW2_BOUNDARY;
	private final Scalar RED_HIGH_2 = _config.RED_HSV_HIGH2_BOUNDARY;

	private final Scalar BLUE_LOW = _config.BLUE_HSV_LOW_BOUNDARY;
	private final Scalar BLUE_HIGH = _config.BLUE_HSV_HIGH_BOUNDARY;


	private final Rect RIGHT_RECT = new Rect(
		new Point(_config.RIGHT_X_POS - _config.RECT_SIZE, _config.RIGHT_Y_POS - _config.RECT_SIZE),
		new Point(_config.RIGHT_X_POS + _config.RECT_SIZE, _config.RIGHT_Y_POS + _config.RECT_SIZE)
	);
	private final Rect CENTER_RECT = new Rect(
		new Point(_config.CENTER_X_POS - _config.RECT_SIZE, _config.CENTER_Y_POS - _config.RECT_SIZE),
		new Point(_config.CENTER_X_POS + _config.RECT_SIZE, _config.CENTER_Y_POS + _config.RECT_SIZE)
	);
	private final Rect LEFT_RECT = new Rect(
		new Point(_config.LEFT_X_PIS - _config.RECT_SIZE, _config.LEFT_Y_POS - _config.RECT_SIZE),
		new Point(_config.LEFT_X_PIS + _config.RECT_SIZE, _config.LEFT_Y_POS + _config.RECT_SIZE)
	);

	private final Scalar RED_COLOR = new Scalar(255.0d, 0.0d, 0.0d);
	private final Scalar BLUE_COLOR = new Scalar(0.0d, 0.0d, 255.0d);


	public int _location = -1;


	public pipeline(final Boolean _isRed) {
		isRed = _isRed;
	}

	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

		if (isRed) {
			Core.inRange(mat, RED_LOW_1, RED_HIGH_1, lowMask);
			Core.inRange(mat, RED_LOW_2, RED_HIGH_2, highMask);
			Core.add(lowMask, highMask, full);
		} else {
			Core.inRange(mat, BLUE_LOW, BLUE_HIGH, full);
		}

		Imgproc.cvtColor(full, mat, Imgproc.COLOR_GRAY2RGB);
		Imgproc.rectangle(mat, RIGHT_RECT, isRed ? RED_COLOR : BLUE_COLOR);
		Imgproc.rectangle(mat, CENTER_RECT, isRed ? RED_COLOR : BLUE_COLOR);
		Imgproc.rectangle(mat, LEFT_RECT, isRed ? RED_COLOR : BLUE_COLOR);

		getLocation();

		lowMask.release();
		highMask.release();
		full.release();
		return mat;
	}

	public void getLocation() {
		double rightConfidence = Core.mean(new Mat(mat, RIGHT_RECT)).val[0];
		double centerConfidence = Core.mean(new Mat(mat, CENTER_RECT)).val[0];
		double leftConfidence = Core.mean(new Mat(mat, LEFT_RECT)).val[0];

		if (rightConfidence > leftConfidence && rightConfidence > centerConfidence)
			_location = 1;
		else if (centerConfidence > rightConfidence && centerConfidence > leftConfidence)
			_location = 2;
		else if (leftConfidence > rightConfidence && leftConfidence > centerConfidence)
			_location = 3;

		if (rightConfidence < 0.3d && centerConfidence < 0.3d && leftConfidence < 0.3d)
			_location = -1;
	}
}