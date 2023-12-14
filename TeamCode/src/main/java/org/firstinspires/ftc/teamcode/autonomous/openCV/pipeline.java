package org.firstinspires.ftc.teamcode.autonomous.openCV;

import org.firstinspires.ftc.teamcode.autonomous.config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
	private final boolean isRed;
	private final config _config = new config();

	private final Mat output = new Mat();


	public pipeline(final boolean _isRed) {
		isRed = _isRed;
	}

	@Override
	public Mat processFrame(final Mat input) {
		Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);

		Core.inRange(output, _config.RED_HSV_LOW1_BOUNDARY, _config.RED_HSV_HIGH1_BOUNDARY, output);
		Core.inRange(output, _config.RED_HSV_LOW2_BOUNDARY, _config.RED_HSV_HIGH2_BOUNDARY, output);

		return input;
	}
}
