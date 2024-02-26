package org.firstinspires.ftc.teamcode.main.arm;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.pid;

import java.util.HashMap;

@Config("Arm config")
public class arm extends config {
	private final motorUtil motorUtil;
	private final pid pid;

	private double armPower = 0d;
	private double armFixPower = 0d;
	private double liftPower = 0d;
	private double liftFixPower = 0d;

	private double armFinalMaxPower = ARM_SPEED;
	private double liftFinalMaxPower = LIFT_SPEED;

	private int armPosition = 0;
	private int liftPosition = 0;

	private double lastArmPower = 0d;
	private int armTargetPosition = 0;
	private boolean armHold = false;
	private boolean lastDpadUp = false;

	private double lastLiftPower = 0d;
	private int liftTargetPosition = 0;


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

		if (GAMEPAD.dpad_up && !this.lastDpadUp) {
			this.armHold = !this.armHold;
			this.armTargetPosition = this.armPosition;
		}
		this.lastDpadUp = GAMEPAD.dpad_up;

		if (this.armPower == 0 && this.lastArmPower != this.armPower)
			this.armTargetPosition = this.armPosition;
		else if (this.armPower == 0 && this.armHold)
			this.armFixPower = this.pid.positionController(this.armPosition, this.armTargetPosition);
		this.lastArmPower = this.armPower;


		if (this.liftPower == 0 && this.lastLiftPower != this.liftPower
//			&& this.liftPosition > MAX_LIFT_POSITION
		)
//			this.liftTargetPosition = Math.max(this.liftPosition, MAX_LIFT_POSITION);
			this.liftTargetPosition = this.liftPosition;
		else if (this.liftPower == 0)
			this.liftFixPower = this.pid.positionController(this.liftPosition, this.liftTargetPosition);
		this.lastLiftPower = this.liftPower;


		if (GAMEPAD.right_bumper) this.armFinalMaxPower = ARM_BOOST;
		else this.armFinalMaxPower = ARM_SPEED;

		if (GAMEPAD.left_bumper) this.liftFinalMaxPower = LIFT_BOOST;
		else this.liftFinalMaxPower = LIFT_SPEED;


		this.armPower -= this.armHold ? this.armFixPower : 0d;
		this.liftPower -= this.liftFixPower;

		this.armPower = this.armPower > -MIN_INPUT_POWER && this.armPower < MIN_INPUT_POWER ? 0d : this.armPower;
		this.liftPower = this.liftPower > -MIN_INPUT_POWER && this.liftPower < MIN_INPUT_POWER ? 0d : this.liftPower;

		this.armPower = Range.clip(this.armPower, -this.armFinalMaxPower, this.armFinalMaxPower);
		this.liftPower = Range.clip(this.liftPower, -this.liftFinalMaxPower, this.liftFinalMaxPower);


		if (GAMEPAD.y) {
			this.motorUtil.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
			this.motorUtil.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		}

//		if (this.liftPosition < MAX_LIFT_POSITION)
//			this.liftPower = 0.2d;


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
		TELEMETRY.addData("Arm hold: ", this.armHold);
		TELEMETRY.addData("Arm fix ", "%.3f", this.armFixPower);
		TELEMETRY.addData("Lift: ", "%.3f", this.liftPower);
		TELEMETRY.addData("Lift fix: ", "%.3f", this.liftFixPower);
		TELEMETRY.addLine();

		this.pid.telemetry(TELEMETRY);

		TELEMETRY.addLine("Speed multipliers");
		TELEMETRY.addData("Arm: ", "%.3f", this.armFinalMaxPower);
		TELEMETRY.addData("Lift: ", "%.3f", this.liftFinalMaxPower);
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