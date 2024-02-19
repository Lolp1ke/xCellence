package org.firstinspires.ftc.teamcode.autonomous.hand;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sigma.hand.config;
import org.firstinspires.ftc.teamcode.utils.servoUtil;

public class hand extends config {
	private final servoUtil servoUtil;


	private double rightClawPosition = CLAW_CLOSE;
	private double leftClawPosition = CLAW_CLOSE;
	private WRIST wrist = WRIST.SCORE;
	private double wristPosition = WRIST_SCORE;

	public hand(final HardwareMap HARDWARE_MAP) {
		this.servoUtil = new servoUtil(
			HARDWARE_MAP.get(Servo.class, "right_claw"),
			HARDWARE_MAP.get(Servo.class, "left_claw"),
			HARDWARE_MAP.get(Servo.class, "wrist")
		);

		this.servoUtil.setDirection(
			Servo.Direction.FORWARD,
			Servo.Direction.REVERSE,
			Servo.Direction.REVERSE
		);
	}

	private void setPositions() {
		this.servoUtil.setPosition(
			this.rightClawPosition,
			this.leftClawPosition,
			this.wristPosition
		);
	}

	public void claw(final boolean IS_LEFT, final CLAW POSITION) {
		this.rightClawPosition = IS_LEFT ? this.rightClawPosition : (POSITION == CLAW.OPEN ? CLAW_OPEN : CLAW_CLOSE);
		this.leftClawPosition = !IS_LEFT ? this.leftClawPosition : (POSITION == CLAW.OPEN ? CLAW_OPEN : CLAW_CLOSE);


		this.setPositions();
	}

	public void wrist(final WRIST wrist) {
		this.wrist = wrist;


		if (this.wrist == WRIST.SCORE)
			this.wristPosition = WRIST_SCORE;
		else if (this.wrist == WRIST.GROUND)
			this.wristPosition = WRIST_GROUND;
		else if (this.wrist == WRIST.MID)
			this.wristPosition = WRIST_MID;

		this.setPositions();
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Hand");
		TELEMETRY.addData("Right claw: ", "%.3f", this.rightClawPosition);
		TELEMETRY.addData("Left claw: ", "%.3f", this.leftClawPosition);
		TELEMETRY.addData("Wrist: ", "%.3f", this.wristPosition);
		TELEMETRY.addLine();


		TELEMETRY.addLine();
	}
}
