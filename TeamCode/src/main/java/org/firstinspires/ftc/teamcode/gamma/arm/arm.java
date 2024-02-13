package org.firstinspires.ftc.teamcode.gamma.arm;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.motorUtil;

import java.util.HashMap;

public class arm extends config {
	private final motorUtil motorUtil;

	private double armPower = 0d;
	private double liftPower = 0d;

	private double armSpeedMultiplier = ARM_SPEED;
	private double liftSpeedMultiplier = LIFT_SPEED;

	private int armPosition = 0;
	private int liftPosition = 0;

	private double lastArmPower = 0d;
	private int armTargetPosition = 0;

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

		this.motorUtil.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		this.motorUtil.setZeroPowerBehaviour(DcMotorEx.ZeroPowerBehavior.BRAKE);
	}

	public void run(final Gamepad GAMEPAD) {
		this.armPower = -GAMEPAD.left_stick_y;
		this.liftPower = -GAMEPAD.right_stick_y;

		HashMap<Integer, Integer> positions = this.motorUtil.getCurrentPositions();

		this.armPosition = (positions.get(0) + positions.get(1)) / 2;
		this.liftPosition = positions.get(2);

		if (this.armPower == 0 && this.lastArmPower != this.armPower)
			this.armTargetPosition = this.armPosition;
		else if (this.armPower == 0) {
			this.motorUtil.setTargetPosition(
				this.armTargetPosition,
				this.armTargetPosition
			);

			this.motorUtil.setMode(
				DcMotorEx.RunMode.RUN_TO_POSITION,
				DcMotorEx.RunMode.RUN_TO_POSITION
			);
		} else this.motorUtil.setMode(
			DcMotorEx.RunMode.RUN_USING_ENCODER,
			DcMotorEx.RunMode.RUN_USING_ENCODER
		);
		this.lastArmPower = this.armPower;

		if (GAMEPAD.right_bumper) this.armSpeedMultiplier = ARM_BOOST;
		else this.armSpeedMultiplier = ARM_SPEED;

		if (GAMEPAD.left_bumper) this.liftSpeedMultiplier = LIFT_SPEED;
		else this.liftSpeedMultiplier = LIFT_BOOST;

		Range.clip(this.armPower, -this.armSpeedMultiplier, this.armSpeedMultiplier);
		Range.clip(this.liftPower, -this.liftSpeedMultiplier, this.liftSpeedMultiplier);

		if (this.motorUtil.isBusy())
			this.motorUtil.setPower(
				0.5d,
				0.5d
			);
		else this.motorUtil.setPower(
			this.armPower,
			this.armPower,
			this.liftPower
		);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Power");
		TELEMETRY.addData("Arm: ", this.armPower);
		TELEMETRY.addData("Lift: ", this.liftPower);
		TELEMETRY.addLine();

		TELEMETRY.addLine("Speed multipliers");
		TELEMETRY.addData("Arm: ", this.armSpeedMultiplier);
		TELEMETRY.addData("Lift: ", this.liftSpeedMultiplier);
		TELEMETRY.addLine();

		TELEMETRY.addLine("Positions");
		TELEMETRY.addData("Arm: ", this.armPosition);
		TELEMETRY.addData("Lift: ", this.liftPosition);
		TELEMETRY.addData("Arm target: ", this.armTargetPosition);
		TELEMETRY.addLine();
	}
}
