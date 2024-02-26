package org.firstinspires.ftc.teamcode.autonomous.openCV;

import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.BLUE_HSV_HIGH_BOUNDARY;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.BLUE_HSV_LOW_BOUNDARY;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.CENTER_RECT_BLUE;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.CENTER_RECT_RED;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.CONFIDENCE;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.LEFT_RECT_BLUE;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.LEFT_RECT_RED;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.RED_HSV_HIGH1_BOUNDARY;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.RED_HSV_HIGH2_BOUNDARY;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.RED_HSV_LOW1_BOUNDARY;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.RED_HSV_LOW2_BOUNDARY;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.RIGHT_RECT_BLUE;
import static org.firstinspires.ftc.teamcode.autonomous.openCV.config.RIGHT_RECT_RED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class pipeline extends OpenCvPipeline {
	private final boolean isRed;

	private final Mat output = new Mat();

	private double rightConfidence = 0d;
	private double centerConfidence = 0d;
	private double leftConfidence = 0d;

	public int location = 3;

	private final Rect rightRect;
	private final Rect centerRect;
	private final Rect leftRect;

	private final Scalar rectColor;

	public pipeline(final boolean isRed) {
		this.isRed = isRed;

		if (this.isRed) {
			this.rightRect = RIGHT_RECT_RED;
			this.centerRect = CENTER_RECT_RED;
			this.leftRect = LEFT_RECT_RED;

			this.rectColor = new Scalar(255d, 0d, 0d);
		} else {
			this.rightRect = RIGHT_RECT_BLUE;
			this.centerRect = CENTER_RECT_BLUE;
			this.leftRect = LEFT_RECT_BLUE;

			this.rectColor = new Scalar(0d, 0d, 255d);
		}
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
				RED_HSV_LOW1_BOUNDARY, RED_HSV_HIGH1_BOUNDARY,
				lowMask
			);
			Core.inRange(
				this.output,
				RED_HSV_LOW2_BOUNDARY, RED_HSV_HIGH2_BOUNDARY,
				highMask
			);

			Core.add(lowMask, highMask, full);

			lowMask.release();
			highMask.release();
		} else Core.inRange(
			this.output,
			BLUE_HSV_LOW_BOUNDARY, BLUE_HSV_HIGH_BOUNDARY,
			full
		);

		Imgproc.cvtColor(full, this.output, Imgproc.COLOR_GRAY2RGB);
		Imgproc.rectangle(
			this.output,
			this.rightRect,
			this.rectColor
		);
		Imgproc.rectangle(
			this.output,
			this.centerRect,
			this.rectColor
		);
		Imgproc.rectangle(
			this.output,
			this.leftRect,
			this.rectColor
		);

		this.setLocation();

		full.release();
		return this.output;
	}

	private void setLocation() {
//		this.rightConfidence = Core.mean(
//			new Mat(this.output, this.rightRect)
//		).val[0] / 255d * 100d;
		this.centerConfidence = Core.mean(
			new Mat(this.output, this.centerRect)
		).val[0] / 255d * 100d;
		this.leftConfidence = Core.mean(
			new Mat(this.output, this.leftRect)
		).val[0] / 255d * 100d;

		if (this.rightConfidence > this.centerConfidence && this.rightConfidence > this.leftConfidence && this.rightConfidence > CONFIDENCE)
			this.location = 3;
		else if (this.centerConfidence > this.rightConfidence && this.centerConfidence > this.leftConfidence && this.centerConfidence > CONFIDENCE)
			this.location = 2;
		else if (this.leftConfidence > this.rightConfidence && this.leftConfidence > this.centerConfidence && this.leftConfidence > CONFIDENCE)
			this.location = 1;
		else this.location = 3;
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Location data");
		TELEMETRY.addData("Current location: ", location);
		TELEMETRY.addData("Right (3): ", rightConfidence);
		TELEMETRY.addData("Center (2): ", centerConfidence);
		TELEMETRY.addData("Left (1): ", leftConfidence);
		TELEMETRY.addLine();
	}
}
