package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class pid {
	private final double P_GAIN;
	private final double I_GAIN;
	private final double D_GAIN;
	private final double MAX_SPEED;

	private final double THRESHOLD_ERROR = 1.5d;
	private final double MAX_TOTAL_ERROR = 1000d;

	private double currentError = 0d;
	private double lastError = 0d;
	private double totalError = 0d;
	private double finalError = 0d;


	public pid(
		final double P_GAIN,
		final double I_GAIN,
		final double D_GAIN,
		final double MAX_SPEED
	) {
		this.P_GAIN = P_GAIN;
		this.I_GAIN = I_GAIN;
		this.D_GAIN = D_GAIN;

		this.MAX_SPEED = MAX_SPEED;
	}

	public double positionContoller(final double CURRENT, final double TARGET) {
		this.currentError = CURRENT - TARGET;

		if (Math.abs(this.currentError) < this.THRESHOLD_ERROR) return 0d;

		this.finalError = this.currentError * this.P_GAIN
			+ this.totalError * this.I_GAIN
			+ (this.currentError - this.lastError) * this.D_GAIN;

		this.lastError = this.currentError;
		this.totalError += this.currentError;

		if (Math.abs(this.totalError) > this.MAX_TOTAL_ERROR)
			this.totalError = this.MAX_TOTAL_ERROR * (Math.abs(this.totalError) / this.totalError);

		return Range.clip(this.finalError, -MAX_SPEED, MAX_SPEED);
	}

	public double headingController(final double CURRENT, final double TARGET) {
		this.currentError = CURRENT - TARGET;

		while (this.currentError > 180d) this.currentError -= 360d;
		while (this.currentError <= -180d) this.currentError += 360d;

		if (Math.abs(this.currentError) < this.THRESHOLD_ERROR) return 0d;

		this.finalError = this.currentError * this.P_GAIN
			+ this.totalError * this.I_GAIN
			+ (this.currentError - this.lastError) * this.D_GAIN;

		this.lastError = this.currentError;
		this.totalError += this.currentError;

		if (Math.abs(this.totalError) > this.MAX_TOTAL_ERROR)
			this.totalError = this.MAX_TOTAL_ERROR * (Math.abs(this.totalError) / this.totalError);

		return Range.clip(this.finalError, -this.MAX_SPEED, this.MAX_SPEED);
	}

	public void reset() {
		this.currentError = 0d;
		this.lastError = 0d;
		this.totalError = 0d;
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addLine("Errors");
		telemetry.addData("Current: ", "%.3f", this.currentError);
		telemetry.addData("Last: ", "%.3f", this.lastError);
		telemetry.addData("Total: ", "%.3f", this.totalError);
		telemetry.addData("Final: ", "%.3f", this.finalError);
		telemetry.addLine();
	}
}
