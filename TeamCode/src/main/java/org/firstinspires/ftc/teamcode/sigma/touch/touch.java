package org.firstinspires.ftc.teamcode.sigma.touch;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class touch extends config {
	private final TouchSensor rightClaw;
	private final TouchSensor leftClaw;

	private final Servo flag;

	private boolean rightClawPressed = false;
	private boolean leftClawPressed = false;

	private double flagPosition = 0d;


	public touch(final HardwareMap HARDWARE_MAP) {
		this.rightClaw = HARDWARE_MAP.get(TouchSensor.class, "right_touch");
		this.leftClaw = HARDWARE_MAP.get(TouchSensor.class, "left_touch");

		this.flag = HARDWARE_MAP.get(Servo.class, "flag");

		this.flag.setDirection(Servo.Direction.REVERSE);
	}

	public void run() {
		this.rightClawPressed = this.rightClaw.isPressed();
		this.leftClawPressed = this.leftClaw.isPressed();

		if (this.rightClawPressed || this.leftClawPressed) this.flagPosition = FLAG_CLOSE;
		else this.flagPosition = FLAG_OPEN;


		this.flag.setPosition(this.flagPosition);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Touch");
		TELEMETRY.addLine("Pressed");
		TELEMETRY.addData("Right: ", this.rightClawPressed);
		TELEMETRY.addData("Left: ", this.leftClawPressed);
		TELEMETRY.addLine();

		TELEMETRY.addLine("Flag");
		TELEMETRY.addData("Position: ", this.flagPosition);

		TELEMETRY.addLine();
	}
}
