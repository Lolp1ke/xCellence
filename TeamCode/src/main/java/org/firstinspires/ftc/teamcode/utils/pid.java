package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class pid {
	private double P_GAIN;
	private double I_GAIN;
	private double D_GAIN;
	private double MAX_SPEED;

	private int THRESHOLD_ERROR;
	private double MAX_TOTAL_ERROR = 100d;

	private double currentError = 0d;
	private double lastError = 0d;
	private double totalError = 0d;
	private double finalError = 0d;

	private double lastTime = 0d;
	private double period = 0d;

	public pid(
		final double P_GAIN,
		final double I_GAIN,
		final double D_GAIN,
		final double MAX_SPEED,
		final int THRESHOLD_ERROR
	) {
		this.P_GAIN = P_GAIN;
		this.I_GAIN = I_GAIN;
		this.D_GAIN = D_GAIN;

		this.MAX_SPEED = MAX_SPEED;
		this.THRESHOLD_ERROR = THRESHOLD_ERROR;
	}

	public void update(final double P_GAIN,
		final double I_GAIN,
		final double D_GAIN,
		final double MAX_SPEED,
		final int THRESHOLD_ERROR
	) {
		this.P_GAIN = P_GAIN;
		this.I_GAIN = I_GAIN;
		this.D_GAIN = D_GAIN;

		this.MAX_SPEED = MAX_SPEED;
		this.THRESHOLD_ERROR = THRESHOLD_ERROR;
	}

	public double positionController(final double CURRENT, final double TARGET) {
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

	public double headingControllerNew(final double CURRENT, final double TARGET) {
		final double currentTime = System.nanoTime() / 1E9d;
		if (this.lastTime == 0d) this.lastTime = currentTime;
		this.period = currentTime - this.lastTime;
		this.lastTime = currentTime;


		this.currentError = CURRENT - TARGET;
		while (this.currentError > 180d) this.currentError -= 360d;
		while (this.currentError <= -180d) this.currentError += 360d;

		final double errorD;
		if (Math.abs(period) > 1E-6) errorD = (this.currentError - this.lastError) / period;
		else errorD = 0;
		this.lastError = this.currentError;

		this.totalError += period * (TARGET - CURRENT);
		this.totalError = totalError < -this.MAX_TOTAL_ERROR
			? -this.MAX_TOTAL_ERROR
			: Math.min(this.MAX_TOTAL_ERROR, totalError);

		this.finalError
			= this.P_GAIN * this.currentError + this.I_GAIN * this.totalError + this.D_GAIN * errorD;

		return Range.clip(this.finalError, -MAX_SPEED, MAX_SPEED);
	}

	public double headingController(final double CURRENT, final double TARGET) {
		this.currentError = CURRENT - TARGET;
		while (this.currentError > 180d) this.currentError -= 360d;
		while (this.currentError <= -180d) this.currentError += 360d;

		if (Math.abs(this.currentError) <= this.THRESHOLD_ERROR) return 0d;

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
		this.finalError = 0d;
	}

	public void setMaxTotalError(final double MAX_TOTAL_ERROR) {
		this.MAX_TOTAL_ERROR = MAX_TOTAL_ERROR;
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("PID");

		TELEMETRY.addLine("Errors");
		TELEMETRY.addData("Current: ", "%.3f", this.currentError);
		TELEMETRY.addData("Last: ", "%.3f", this.lastError);
		TELEMETRY.addData("Total: ", "%.3f", this.totalError);
		TELEMETRY.addData("Final: ", "%.3f", this.finalError);
		TELEMETRY.addLine();
	}
}
