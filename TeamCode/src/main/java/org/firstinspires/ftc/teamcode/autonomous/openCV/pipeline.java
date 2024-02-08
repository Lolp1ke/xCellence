package org.firstinspires.ftc.teamcode.autonomous.openCV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
	private final config config = new config();
	private final boolean isRed;

	private final Mat output = new Mat();

	private double rightConfidence = 0d;
	private double centerConfidence = 0d;
	private double leftConfidence = 0d;

	public int location = 0;

	public pipeline(final boolean isRed) {
		this.isRed = isRed;
	}

	@Override
	public Mat processFrame(final Mat input) {
		final Mat full = new Mat();
		Imgproc.cvtColor(input, this.output, Imgproc.COLOR_RGB2HSV);

		if (this.isRed) {
			final Mat lowMask = new Mat();
			final Mat highMask = new Mat();

			Core.inRange(
				this.output,
				this.config.RED_HSV_LOW1_BOUNDARY, this.config.RED_HSV_HIGH1_BOUNDARY,
				lowMask
			);
			Core.inRange(
				this.output,
				this.config.RED_HSV_LOW2_BOUNDARY, this.config.RED_HSV_HIGH2_BOUNDARY,
				highMask
			);

			Core.add(lowMask, highMask, full);

			lowMask.release();
			highMask.release();
		} else
			Core.inRange(
				this.output,
				this.config.BLUE_HSV_LOW_BOUNDARY, this.config.BLUE_HSV_HIGH_BOUNDARY,
				full
			);

		Imgproc.cvtColor(full, this.output, Imgproc.COLOR_GRAY2RGB);
		Imgproc.rectangle(
			this.output,
			this.config.RIGHT_RECT,
			this.isRed ? new Scalar(255d, 0d, 0d) : new Scalar(0d, 0d, 255d)
		);
		Imgproc.rectangle(
			this.output,
			this.config.CENTER_RECT,
			this.isRed ? new Scalar(255d, 0d, 0d) : new Scalar(0d, 0d, 255d)
		);
		Imgproc.rectangle(
			this.output,
			this.config.LEFT_RECT,
			this.isRed ? new Scalar(255d, 0d, 0d) : new Scalar(0d, 0d, 255d)
		);

		this.setLocation();

		full.release();
		return this.output;
	}

	private void setLocation() {
		this.rightConfidence = Core.mean(
			new Mat(this.output, this.config.RIGHT_RECT)
		).val[0] / 255d * 100d;
		this.centerConfidence = Core.mean(
			new Mat(this.output, this.config.CENTER_RECT)
		).val[0] / 255d * 100d;
		this.leftConfidence = Core.mean(
			new Mat(this.output, this.config.LEFT_RECT)
		).val[0] / 255d * 100d;

		if (this.rightConfidence > this.centerConfidence && this.rightConfidence > this.leftConfidence && this.rightConfidence > this.config.CONFIDENCE)
			this.location = 3;
		else if (this.centerConfidence > this.rightConfidence && this.centerConfidence > this.leftConfidence && this.centerConfidence > this.config.CONFIDENCE)
			this.location = 2;
		else if (this.leftConfidence > this.rightConfidence && this.leftConfidence > this.centerConfidence && this.leftConfidence > this.config.CONFIDENCE)
			this.location = 1;
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addLine("Location data");
		telemetry.addData("Current location: ", location);
		telemetry.addData("Right (3): ", rightConfidence);
		telemetry.addData("Center (2): ", centerConfidence);
		telemetry.addData("Left (1): ", leftConfidence);
	}
}
