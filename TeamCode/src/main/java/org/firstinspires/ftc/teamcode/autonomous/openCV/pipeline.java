package org.firstinspires.ftc.teamcode.autonomous.openCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
	private final config _config = new config();
	private final Boolean isRed;

	private final Mat output = new Mat();


	public double rightConfidence = 0d;
	public double centerConfidence = 0d;
	public double leftConfidence = 0d;
	public int location = 3;


	public pipeline(final Boolean _isRed) {
		isRed = _isRed;
	}

	@Override
	public Mat processFrame(final Mat input) {
		final Mat full = new Mat();
		Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);

		if (isRed) {
			final Mat lowMask = new Mat();
			final Mat highMask = new Mat();

			Core.inRange(output, _config.RED_HSV_LOW1_BOUNDARY, _config.RED_HSV_HIGH1_BOUNDARY, lowMask);
			Core.inRange(output, _config.RED_HSV_LOW2_BOUNDARY, _config.RED_HSV_HIGH2_BOUNDARY, highMask);
			Core.add(lowMask, highMask, full);

			lowMask.release();
			highMask.release();
		} else {
			Core.inRange(output, _config.BLUE_HSV_LOW_BOUNDARY, _config.BLUE_HSV_HIGH_BOUNDARY, full);
		}

		Imgproc.cvtColor(full, output, Imgproc.COLOR_GRAY2RGB);
		Imgproc.rectangle(output, _config.RIGHT_RECT, isRed ? new Scalar(255d, 0d, 0d) : new Scalar(0d, 0d, 255d));
		Imgproc.rectangle(output, _config.CENTER_RECT, isRed ? new Scalar(255d, 0d, 0d) : new Scalar(0d, 0d, 255d));
		Imgproc.rectangle(output, _config.LEFT_RECT, isRed ? new Scalar(255d, 0d, 0d) : new Scalar(0d, 0d, 255d));

		setLocation();

		full.release();
		return output;
	}

	private void setLocation() {
		final double CONFIDENCE = 30d;
		rightConfidence = Core.mean(new Mat(output, _config.RIGHT_RECT)).val[0] / 255d * 100d;
		centerConfidence = Core.mean(new Mat(output, _config.CENTER_RECT)).val[0] / 255d * 100d;
		leftConfidence = Core.mean(new Mat(output, _config.LEFT_RECT)).val[0] / 255d * 100d;

		if (rightConfidence > centerConfidence && rightConfidence > leftConfidence && rightConfidence > CONFIDENCE)
			location = 3;
		else if (centerConfidence > rightConfidence && centerConfidence > leftConfidence && centerConfidence > CONFIDENCE)
			location = 2;
		else if (leftConfidence > rightConfidence && leftConfidence > centerConfidence && leftConfidence > CONFIDENCE)
			location = 1;
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addLine("Location data");
		telemetry.addData("Current location: ", location);
		telemetry.addData("Right: ", rightConfidence);
		telemetry.addData("Center: ", centerConfidence);
		telemetry.addData("Left: ", leftConfidence);
	}
}