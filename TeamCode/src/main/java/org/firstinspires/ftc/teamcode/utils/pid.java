package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class pid {
	private final double P_HEADING_GAIN;
	private final double I_HEADING_GAIN;
	private final double D_HEADING_GAIN;
	private final double MAX_SPEED;

	private final double MAX_TOTAL_ERROR = 1000d;

	private double currentHeadingError = 0d;
	private double lastHeadingError = 0d;
	private double totalHeadingError = 0d;
	private double finalHeadingError = 0d;

	public pid(
		final double P_HEADING_GAIN,
		final double I_HEADING_GAIN,
		final double D_HEADING_GAIN,
		final double MAX_SPEED
	) {
		this.P_HEADING_GAIN = P_HEADING_GAIN;
		this.I_HEADING_GAIN = I_HEADING_GAIN;
		this.D_HEADING_GAIN = D_HEADING_GAIN;

		this.MAX_SPEED = MAX_SPEED;
	}

	public double headingController(final double CURRENT, final double TARGET) {
		this.currentHeadingError = CURRENT - TARGET;

		while (this.currentHeadingError > 180d) this.currentHeadingError -= 360d;
		while (this.currentHeadingError <= -180d) this.currentHeadingError += 360d;


		this.finalHeadingError = this.currentHeadingError * this.P_HEADING_GAIN
			+ this.totalHeadingError * this.I_HEADING_GAIN
			+ (this.currentHeadingError - this.lastHeadingError) * this.D_HEADING_GAIN;

		this.lastHeadingError = this.currentHeadingError;
		this.totalHeadingError += this.currentHeadingError;

		if (Math.abs(this.totalHeadingError) > this.MAX_TOTAL_ERROR)
			this.totalHeadingError = this.MAX_TOTAL_ERROR * (Math.abs(this.totalHeadingError) / this.totalHeadingError);

		return Range.clip(this.finalHeadingError, -this.MAX_SPEED, this.MAX_SPEED);
	}

	public void reset() {
		this.currentHeadingError = 0d;
		this.lastHeadingError = 0d;
		this.totalHeadingError = 0d;
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addLine("Heading errors");
		telemetry.addData("Current: ", this.currentHeadingError);
		telemetry.addData("Last: ", this.lastHeadingError);
		telemetry.addData("Total: ", this.totalHeadingError);
		telemetry.addData("Final: ", this.finalHeadingError);
		telemetry.addLine();
	}
}
