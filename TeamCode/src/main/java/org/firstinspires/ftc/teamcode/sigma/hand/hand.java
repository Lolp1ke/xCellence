package org.firstinspires.ftc.teamcode.sigma.hand;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.servoUtil;

@Config("Hand c")
public class hand extends config {
	private final servoUtil servoUtil;
	private final motorUtil motorUtil;


	private WRIST wrist = WRIST.SCORE;

	private double wristPosition = WRIST_SCORE;
	private double wristOffset = 0d;
	private double lastRT = 0d;
	private double lastLT = 0d;

	private double rightClawPosition = CLAW_OPEN;
	private double leftClawPosition = CLAW_OPEN;

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

		this.servoUtil.setPosition(
			this.rightClawPosition,
			this.leftClawPosition,
			this.wristPosition
		);


		this.motorUtil = new motorUtil(
			HARDWARE_MAP.get(DcMotorEx.class, "right_arm"),
			HARDWARE_MAP.get(DcMotorEx.class, "left_arm")
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

		if (GAMEPAD.x)
			this.wrist = WRIST.SCORE;
		else if (GAMEPAD.a)
			this.wrist = WRIST.GROUND;
		else if (GAMEPAD.b)
			this.wrist = WRIST.MID;

		this.wristOffset = WRIST_MID * (this.motorUtil.getCurrentPositions().get(0) / 220d);

		if (this.wrist == WRIST.SCORE)
			this.wristPosition = WRIST_SCORE;
		else if (this.wrist == WRIST.GROUND)
			this.wristPosition = WRIST_GROUND;
		else if (this.wrist == WRIST.MID)
			this.wristPosition = WRIST_MID + this.wristOffset;


		this.servoUtil.setPosition(
			this.rightClawPosition,
			this.leftClawPosition,
			this.wristPosition
		);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Servos");
		TELEMETRY.addData("Right claw: ", "%.3f", this.rightClawPosition);
		TELEMETRY.addData("Left claw: ", "%.3f", this.leftClawPosition);
		TELEMETRY.addData("Wrist: ", this.wrist.name());
		TELEMETRY.addData("Wrist position: ", "%.3f", this.wristPosition);
		TELEMETRY.addData("Wrist offset: ", "%.3f", this.wristOffset);
		TELEMETRY.addLine();

		TELEMETRY.addData("Motor position", this.motorUtil.getCurrentPositions().get(0));
		TELEMETRY.addLine();

		TELEMETRY.addLine();
	}
}
