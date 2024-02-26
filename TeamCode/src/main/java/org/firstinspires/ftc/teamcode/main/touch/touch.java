package org.firstinspires.ftc.teamcode.main.touch;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.servoUtil;

import java.util.concurrent.TimeUnit;

@Config("Touch config")
public class touch extends config {
	private final TouchSensor rightClaw;
	private final TouchSensor leftClaw;

	private final servoUtil servoUtil;

	private boolean rightClawPressed = false;
	private boolean lastRightClawPressed = false;
	private boolean leftClawPressed = false;
	private boolean lastLeftClawPressed = false;

	private double rightFlagPosition = 0d;
	private double leftFlagPosition = 0d;

	private final double MAX_FLAG_DURATION = 1d;

	private ElapsedTime rightTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
	private ElapsedTime leftTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


	public touch(final HardwareMap HARDWARE_MAP) {
		this.rightClaw = HARDWARE_MAP.get(TouchSensor.class, "right_touch");
		this.leftClaw = HARDWARE_MAP.get(TouchSensor.class, "left_touch");

		this.servoUtil = new servoUtil(
			HARDWARE_MAP.get(Servo.class, "right_flag"),
			HARDWARE_MAP.get(Servo.class, "left_flag")
		);

		this.servoUtil.setDirection(
			Servo.Direction.REVERSE,
			Servo.Direction.FORWARD
		);

		this.rightTimer.startTime();
	}

	public void run() {
		this.rightClawPressed = this.rightClaw.isPressed();
		this.leftClawPressed = this.leftClaw.isPressed();

//		if (this.rightClawPressed && !this.lastRightClawPressed) {
//			this.rightFlagPosition = FLAG_OPEN;
//			this.rightTimer.reset();
//			this.rightTimer.startTime();
//		} else if (!this.rightClawPressed) this.rightFlagPosition = FLAG_CLOSE;
//		this.lastRightClawPressed = this.rightClawPressed;
//
//
//		if (this.leftClawPressed && !this.lastLeftClawPressed) {
//			this.leftFlagPosition = FLAG_OPEN;
//			this.leftTimer.reset();
//			this.leftTimer.startTime();
//		} else if (!this.leftClawPressed) this.leftFlagPosition = FLAG_CLOSE;
//		this.lastLeftClawPressed = this.leftClawPressed;


		if (this.rightTimer.time(TimeUnit.SECONDS) > this.MAX_FLAG_DURATION && this.rightClawPressed)
			this.rightFlagPosition = FLAG_CLOSE;
		if (this.leftTimer.time(TimeUnit.SECONDS) > this.MAX_FLAG_DURATION && this.leftClawPressed)
			this.leftFlagPosition = FLAG_CLOSE;


		if (!this.rightClawPressed)
			this.rightTimer.reset();
		if (!this.leftClawPressed)
			this.leftTimer.reset();

		this.servoUtil.setPosition(
			this.rightFlagPosition,
			this.leftFlagPosition
		);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Touch sensors");

		TELEMETRY.addLine("Pressed?");
		TELEMETRY.addData("Right: ", this.rightClawPressed);
		TELEMETRY.addData("Left: ", this.leftClawPressed);
		TELEMETRY.addLine();

		TELEMETRY.addLine("Duration");
		TELEMETRY.addData("Right: ", this.rightTimer.time(TimeUnit.SECONDS));
		TELEMETRY.addData("Left: ", this.leftTimer.time(TimeUnit.SECONDS));
		TELEMETRY.addLine();

		TELEMETRY.addLine("Flags");
		TELEMETRY.addData("Right: ", this.rightFlagPosition);
		TELEMETRY.addData("Left: ", this.leftFlagPosition);
		TELEMETRY.addLine();

		TELEMETRY.addLine();
	}
}
