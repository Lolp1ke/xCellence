package org.firstinspires.ftc.teamcode.sigma.arm;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.pid;

import java.util.HashMap;

public class arm extends config {
	private final motorUtil motorUtil;
	private final pid pid;

	private double armPower = 0d;
	private double armFixPower = 0d;
	private double liftPower = 0d;
	private double liftFixPower = 0d;

	private double armSpeedMultiplier = ARM_SPEED;
	private double liftSpeedMultiplier = LIFT_SPEED;

	private int armPosition = 0;
	private int liftPosition = 0;

	private double lastArmPower = 0d;
	private int armTargetPosition = 0;

	private double lastLiftPower = 0d;
	private int liftTargetPosition = 0;

	private final int TARGET_TOLERANCE = 5;

	public arm(final HardwareMap HARDWARE_MAP) {
		this.motorUtil = new motorUtil(
			HARDWARE_MAP.get(DcMotorEx.class, "right_arm"),
			HARDWARE_MAP.get(DcMotorEx.class, "left_arm"),
			HARDWARE_MAP.get(DcMotorEx.class, "lift")
		);

		this.motorUtil.setDirection(
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE,
			DcMotorEx.Direction.FORWARD
		);

		this.motorUtil.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		this.motorUtil.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		this.motorUtil.setZeroPowerBehaviour(DcMotorEx.ZeroPowerBehavior.BRAKE);


		this.pid = new pid(
			P_POSITION_GAIN,
			I_POSITION_GAIN,
			D_POSITION_GAIN,
			MAX_GAIN_SPEED,
			THRESHOLD_ERROR
		);
	}

	public void run(final Gamepad GAMEPAD) {
		final HashMap<Integer, Integer> positions = this.motorUtil.getCurrentPositions();

		this.armPosition = (positions.get(0) + positions.get(1)) / 2;
		this.liftPosition = positions.get(2);

		this.armPower = GAMEPAD.left_stick_y;
		this.liftPower = GAMEPAD.right_stick_y;


		if (this.armPower == 0 && this.lastArmPower != this.armPower)
			this.armTargetPosition = this.armPosition;
		else if (this.armPower == 0
			|| Math.abs(this.armTargetPosition - this.armPosition) > this.TARGET_TOLERANCE
		) this.armFixPower = this.pid.positionController(this.armPosition, this.armTargetPosition);
		this.lastArmPower = this.armPower;


		if (this.liftPower == 0 && this.lastLiftPower != this.armPower)
			this.liftTargetPosition = this.armPosition;
		else if (this.liftPower == 0
			|| Math.abs(this.liftTargetPosition - this.liftPosition) > this.TARGET_TOLERANCE
		) this.liftFixPower = this.pid.positionController(this.liftPosition, this.liftTargetPosition);
		this.lastLiftPower = this.liftPower;


		if (GAMEPAD.right_bumper) this.armSpeedMultiplier = ARM_BOOST;
		else this.armSpeedMultiplier = ARM_SPEED;

		if (GAMEPAD.left_bumper) this.liftSpeedMultiplier = LIFT_SPEED;
		else this.liftSpeedMultiplier = LIFT_BOOST;


		this.armPower = Range.clip(this.armPower - this.armFixPower,
			-this.armSpeedMultiplier, this.armSpeedMultiplier);
		this.liftPower = Range.clip(this.liftPower - this.liftFixPower, -this.liftSpeedMultiplier, this.liftSpeedMultiplier);

		this.armPower = this.armPower > -0.1d && this.armPower < 0.1d ? 0d : this.armPower;
		this.liftPower = this.liftPower > -0.1d && this.liftPower < 0.1d ? 0d : this.liftPower;


		this.motorUtil.setPower(
			this.armPower,
			this.armPower,
			this.liftPower
		);
	}


	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Arm");

		TELEMETRY.addLine("Power");
		TELEMETRY.addData("Arm: ", "%.3f", this.armPower);
		TELEMETRY.addData("Arm fix ", "%.3f", this.armFixPower);
		TELEMETRY.addData("Lift: ", "%.3f", this.liftPower);
		TELEMETRY.addData("Lift fix: ", "%.3f", this.liftFixPower);
		TELEMETRY.addLine();

		this.pid.telemetry(TELEMETRY);

		TELEMETRY.addLine("Speed multipliers");
		TELEMETRY.addData("Arm: ", "%.3f", this.armSpeedMultiplier);
		TELEMETRY.addData("Lift: ", "%.3f", this.liftSpeedMultiplier);
		TELEMETRY.addLine();

		TELEMETRY.addLine("Positions");
		TELEMETRY.addData("Arm: ", this.armPosition);
		TELEMETRY.addData("Arm target: ", this.armTargetPosition);
		TELEMETRY.addData("Lift: ", this.liftPosition);
		TELEMETRY.addData("Lift target: ", this.liftTargetPosition);
		TELEMETRY.addLine();

		TELEMETRY.addLine();
	}
}