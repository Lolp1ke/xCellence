package org.firstinspires.ftc.teamcode.main.movement;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.imuUtil;
import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.pid;

@Config("Movement config")
public class movement extends config {
	private final motorUtil motorUtil;
	private final pid pid;
	private final imuUtil imuUtil;


	private double finalMaxSpeed = SPEED;
	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;
	private double fix = 0d;

	private boolean holdHeading = true;
	private double targetHeading;
	private double lastTurn = 0d;

	private DRIVE_MODE driveMode = DRIVE_MODE.ROBOT;

	public movement(final HardwareMap HARDWARE_MAP) {
		this.motorUtil = new motorUtil(
			HARDWARE_MAP.get(DcMotorEx.class, "right_rear"),
			HARDWARE_MAP.get(DcMotorEx.class, "left_rear"),
			HARDWARE_MAP.get(DcMotorEx.class, "right_front"),
			HARDWARE_MAP.get(DcMotorEx.class, "left_front")
		);

		this.motorUtil.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		this.motorUtil.setZeroPowerBehaviour(DcMotorEx.ZeroPowerBehavior.BRAKE);

		this.motorUtil.setDirection(
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE,
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE
		);


		this.imuUtil = new imuUtil(HARDWARE_MAP.get(IMU.class, "imu"));
		this.imuUtil.initialize(
			RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
			RevHubOrientationOnRobot.UsbFacingDirection.UP
		);
		this.imuUtil.reset();
		this.targetHeading = -this.imuUtil.get(AngleUnit.RADIANS);


		this.pid = new pid(
			P_HEADING_GAIN,
			I_HEADING_GAIN,
			D_HEADING_GAIN,
			PID_MAX_SPEED,
			THRESHOLD_ERROR
		);
	}

	public void run(final Gamepad GAMEPAD) {
		if (GAMEPAD.dpad_up && GAMEPAD.right_trigger > 0.3d)
			this.driveMode = DRIVE_MODE.ROBOT;
		else if (GAMEPAD.dpad_down && GAMEPAD.right_trigger > 0.3d)
			this.driveMode = DRIVE_MODE.FIELD;

		if (this.driveMode == DRIVE_MODE.ROBOT)
			this.robotCentric(GAMEPAD);
		else this.fieldCentric(GAMEPAD);
	}

	private void robotCentric(final Gamepad GAMEPAD) {
		this.finalMaxSpeed = GAMEPAD.left_bumper ?
			DECELERATION : SPEED;

		final double x = GAMEPAD.left_stick_x;
		final double y = -GAMEPAD.left_stick_y;
		final double turn = GAMEPAD.right_stick_x;

		final double angle = Math.atan2(y, x);
		final double power = Math.hypot(x, y);

		final double sin = Math.sin(angle - Math.PI / 4);
		final double cos = Math.cos(angle - Math.PI / 4);
		final double max = Math.max(Math.abs(sin), Math.abs(cos));

		if (turn == 0 && this.lastTurn != turn) {
			this.holdHeading = true;
			this.targetHeading = -this.imuUtil.get(AngleUnit.RADIANS);
			this.pid.reset();
		} else if (turn != 0) this.holdHeading = false;
		this.lastTurn = turn;


		this.pid.update(
			P_HEADING_GAIN,
			I_HEADING_GAIN,
			D_HEADING_GAIN,
			PID_MAX_SPEED,
			THRESHOLD_ERROR
		);
		if (this.holdHeading)
			this.fix = this.pid.headingControllerNew(Math.toDegrees(-this.targetHeading), this.imuUtil.get());
		else this.fix = 0d;

//		this.fix = this.fix > -0.3d && this.fix < 0.3d ? 0 : this.fix;

		this.rightRearPower = power * cos / max - turn + this.fix;
		this.leftRearPower = power * sin / max + turn - this.fix;
		this.rightFrontPower = power * sin / max - turn + this.fix;
		this.leftFrontPower = power * cos / max + turn - this.fix;

		this.rightRearPower = this.rightRearPower > -MIN_INPUT_POWER && this.rightRearPower < MIN_INPUT_POWER ? 0d : this.rightRearPower;
		this.leftRearPower = this.leftRearPower > -MIN_INPUT_POWER && this.leftRearPower < MIN_INPUT_POWER ? 0d : this.leftRearPower;
		this.rightFrontPower = this.rightFrontPower > -MIN_INPUT_POWER && this.rightFrontPower < MIN_INPUT_POWER ? 0d : this.rightFrontPower;
		this.leftFrontPower = this.leftFrontPower > -MIN_INPUT_POWER && this.leftFrontPower < MIN_INPUT_POWER ? 0d : this.leftFrontPower;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.finalMaxSpeed, this.finalMaxSpeed);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.finalMaxSpeed, this.finalMaxSpeed);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.finalMaxSpeed, this.finalMaxSpeed);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.finalMaxSpeed, this.finalMaxSpeed);


		this.motorUtil.setPower(
			this.rightRearPower,
			this.leftRearPower,
			this.rightFrontPower,
			this.leftFrontPower
		);
	}

	private void fieldCentric(final Gamepad GAMEPAD) {
		final double x = GAMEPAD.left_stick_x;
		final double y = -GAMEPAD.left_stick_y;
		final double turn = GAMEPAD.right_stick_x;

		this.finalMaxSpeed = GAMEPAD.left_bumper
			? DECELERATION : SPEED;

		final double heading = -this.imuUtil.get(AngleUnit.RADIANS);

		if (GAMEPAD.a) {
			this.imuUtil.reset();
			this.targetHeading = -this.imuUtil.get(AngleUnit.RADIANS);
		}

		if (turn == 0d && this.lastTurn != turn) {
			this.holdHeading = true;
			this.targetHeading = -this.imuUtil.get(AngleUnit.RADIANS);
			this.pid.reset();
		} else if (turn != 0) this.holdHeading = false;
		this.lastTurn = turn;

		this.pid.update(
			P_HEADING_GAIN,
			I_HEADING_GAIN,
			D_HEADING_GAIN,
			PID_MAX_SPEED,
			THRESHOLD_ERROR
		);
		if (this.holdHeading && this.finalMaxSpeed != DECELERATION)
			this.fix = this.pid.headingControllerNew(Math.toDegrees(-this.targetHeading), this.imuUtil.get());
		else this.fix = 0d;

		final double rotX = x * Math.cos(heading) - y * Math.sin(heading);
		final double rotY = x * Math.sin(heading) + y * Math.cos(heading);

		this.rightRearPower = rotY + rotX - turn + this.fix;
		this.leftRearPower = rotY - rotX + turn - this.fix;
		this.rightFrontPower = rotY - rotX - turn + this.fix;
		this.leftFrontPower = rotY + rotX + turn - this.fix;

		this.rightRearPower = this.rightRearPower > -MIN_INPUT_POWER && this.rightRearPower < MIN_INPUT_POWER ? 0d : this.rightRearPower;
		this.leftRearPower = this.leftRearPower > -MIN_INPUT_POWER && this.leftRearPower < MIN_INPUT_POWER ? 0d : this.leftRearPower;
		this.rightFrontPower = this.rightFrontPower > -MIN_INPUT_POWER && this.rightFrontPower < MIN_INPUT_POWER ? 0d : this.rightFrontPower;
		this.leftFrontPower = this.leftFrontPower > -MIN_INPUT_POWER && this.leftFrontPower < MIN_INPUT_POWER ? 0d : this.leftFrontPower;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.finalMaxSpeed, this.finalMaxSpeed);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.finalMaxSpeed, this.finalMaxSpeed);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.finalMaxSpeed, this.finalMaxSpeed);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.finalMaxSpeed, this.finalMaxSpeed);


		this.motorUtil.setPower(
			this.rightRearPower,
			this.leftRearPower,
			this.rightFrontPower,
			this.leftFrontPower
		);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Movement");

		TELEMETRY.addLine("Power");
		TELEMETRY.addData("Multiplier: ", "%.3f", this.finalMaxSpeed);
		TELEMETRY.addData("Right rear: ", "%.3f", this.rightRearPower);
		TELEMETRY.addData("Left rear: ", "%.3f", this.leftRearPower);
		TELEMETRY.addData("Right front: ", "%.3f", this.rightFrontPower);
		TELEMETRY.addData("Left front: ", "%.3f", this.leftFrontPower);
		TELEMETRY.addData("Fix: ", "%.3f", this.fix);
		TELEMETRY.addLine();

		TELEMETRY.addLine("Heading");
		TELEMETRY.addData("Hold: ", this.holdHeading);
		TELEMETRY.addData("Target: ", "%.3f", this.targetHeading * 180d / Math.PI);
		TELEMETRY.addData("Current: ", "%.3f", this.imuUtil.get());
		TELEMETRY.addLine();

		this.pid.telemetry(TELEMETRY);

		TELEMETRY.addLine();
	}
}
