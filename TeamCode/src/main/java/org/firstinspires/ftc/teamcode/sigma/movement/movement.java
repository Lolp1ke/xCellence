package org.firstinspires.ftc.teamcode.sigma.movement;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.imuUtil;
import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.pid;

public class movement extends config {
	private final motorUtil motorUtil;
	private final pid pid;
	private final imuUtil imuUtil;


	private double speedMultiplier = SPEED;
	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;
	private double fix = 0d;

	private boolean holdHeading = true;
	private double targetHeading;
	private double lastTurn = 0d;

	private DRIVE_MODE driveMode = DRIVE_MODE.FIELD;

	public movement(final HardwareMap HARDWARE_MAP) {
		final VoltageSensor voltageSensor = HARDWARE_MAP.voltageSensor.iterator().next();

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

		this.motorUtil.setVelocityPIDFCoefficients(
			P_VELOCITY_GAIN,
			I_VELOCITY_GAIN,
			D_VELOCITY_GAIN,
			F_VELOCITY_GAIN,
			voltageSensor.getVoltage()
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
		this.speedMultiplier = GAMEPAD.left_bumper ?
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

		if (this.holdHeading)
			this.fix = this.pid.headingController(Math.toDegrees(-this.targetHeading), this.imuUtil.get());
		else this.fix = 0d;

		this.fix = this.fix > -0.3d && this.fix < 0.3d ? 0 : this.fix;

		this.rightRearPower = power * cos / max - turn + this.fix;
		this.leftRearPower = power * sin / max + turn - this.fix;
		this.rightFrontPower = power * sin / max - turn + this.fix;
		this.leftFrontPower = power * cos / max + turn - this.fix;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.speedMultiplier, this.speedMultiplier);

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

		this.speedMultiplier = GAMEPAD.left_bumper
			? DECELERATION : SPEED;

		final double heading = -this.imuUtil.get(AngleUnit.RADIANS);

		if (GAMEPAD.a) {
			this.imuUtil.reset();
			this.targetHeading = -this.imuUtil.get(AngleUnit.RADIANS);
		}

		if (turn == 0 && this.lastTurn != turn) {
			this.holdHeading = true;
			this.targetHeading = -this.imuUtil.get(AngleUnit.RADIANS);
			this.pid.reset();
		} else if (turn != 0) this.holdHeading = false;
		this.lastTurn = turn;

		if (this.holdHeading)
			this.fix = this.pid.headingController(Math.toDegrees(-this.targetHeading), this.imuUtil.get());
		else this.fix = 0d;

		this.fix = this.fix > -0.15d && this.fix < 0.15d ? 0 : this.fix;

		final double rotX = x * Math.cos(heading) - y * Math.sin(heading);
		final double rotY = x * Math.sin(heading) + y * Math.cos(heading);

		this.rightRearPower = rotY + rotX - turn + this.fix;
		this.leftRearPower = rotY - rotX + turn - this.fix;
		this.rightFrontPower = rotY - rotX - turn + this.fix;
		this.leftFrontPower = rotY + rotX + turn - this.fix;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.speedMultiplier, this.speedMultiplier);

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
		TELEMETRY.addData("Right rear: ", "%.3f", this.rightRearPower);
		TELEMETRY.addData("Left rear: ", "%.3f", this.leftRearPower);
		TELEMETRY.addData("Right front: ", "%.3f", this.rightFrontPower);
		TELEMETRY.addData("Left front: ", "%.3f", this.leftFrontPower);
		TELEMETRY.addData("Fix: ", "%.3f", this.fix);
		TELEMETRY.addLine();

		TELEMETRY.addData("Multiplier: ", "%.3f", this.speedMultiplier);
		TELEMETRY.addLine();

		TELEMETRY.addLine("Heading");
		TELEMETRY.addData("Hold: ", this.holdHeading);
		TELEMETRY.addData("Target: ", "%.3f", this.targetHeading * 180d / Math.PI);
		TELEMETRY.addLine();

		this.pid.telemetry(TELEMETRY);

		TELEMETRY.addLine();
	}
}
