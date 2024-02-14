package org.firstinspires.ftc.teamcode.sigma.hand;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.servoUtil;

public class hand extends config {
	private final servoUtil servoUtil;

	private double wristPosition = WRIST_SCORE;
	private double lastRT = 0d;
	private double lastLT = 0d;

	private double rightClawPosition = CLAW_CLOSE;
	private double leftClawPosition = CLAW_CLOSE;

	public hand(final HardwareMap HARDWARE_MAP) {
		this.servoUtil = new servoUtil(
			HARDWARE_MAP.get(Servo.class, "right_claw"),
			HARDWARE_MAP.get(Servo.class, "left_claw"),
			HARDWARE_MAP.get(Servo.class, "wrist")
		);

		this.servoUtil.setDirection(
			Servo.Direction.FORWARD,
			Servo.Direction.REVERSE,
			Servo.Direction.FORWARD
		);
	}

	public void run(final Gamepad GAMEPAD) {
		final double rt = GAMEPAD.right_trigger;
		final double lt = GAMEPAD.left_trigger;

		if (rt == 0 && this.lastRT != rt)
			this.rightClawPosition = this.rightClawPosition == CLAW_CLOSE
				? CLAW_OPEN
				: CLAW_CLOSE;

		if (lt == 0 && this.lastLT != lt)
			this.leftClawPosition = this.leftClawPosition == CLAW_CLOSE
				? CLAW_OPEN
				: CLAW_CLOSE;

		this.lastRT = rt;
		this.lastLT = lt;

		if (GAMEPAD.x) this.wristPosition = WRIST_SCORE;
		else if (GAMEPAD.a) this.wristPosition = WRIST_GROUND;
		else if (GAMEPAD.b) this.wristPosition = WRIST_MID;

		this.servoUtil.setPosition(
			this.rightClawPosition,
			this.leftClawPosition,
			this.wristPosition
		);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Servos");
		TELEMETRY.addData("Right claw: ", this.rightClawPosition);
		TELEMETRY.addData("Left claw: ", this.leftClawPosition);
		TELEMETRY.addData("Wrist: ", this.wristPosition);
		TELEMETRY.addLine();
	}
}
