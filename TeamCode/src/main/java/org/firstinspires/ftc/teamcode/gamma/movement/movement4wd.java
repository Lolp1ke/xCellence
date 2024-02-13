package org.firstinspires.ftc.teamcode.gamma.movement;

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

public class movement4wd extends config {
	private final motorUtil motorUtil;
	private final imuUtil imuUtil;
	private final pid pid;

	private final VoltageSensor batteryVoltageSensor;


	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;
	private double fix = 0d;

	private boolean holdHeading = true;
	private double targetHeading = 0d;
	private double lastTurn = 0d;

	private double speedMultiplier = SPEED;

	private DRIVE_MODE driveMode = DRIVE_MODE.ROBOT;

	public movement4wd(final HardwareMap HARDWARE_MAP) {
		this.batteryVoltageSensor = HARDWARE_MAP.voltageSensor.iterator().next();


		this.motorUtil = new motorUtil(
			HARDWARE_MAP.get(DcMotorEx.class, "right_rear")
		);

		this.motorUtil.setDirection(
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE,
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE
		);

		this.motorUtil.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

		this.motorUtil.setZeroPowerBehaviour(DcMotorEx.ZeroPowerBehavior.BRAKE);

		this.motorUtil.setVelocityPIDFCoefficients(
			P_VELOCITY_GAIN,
			I_VELOCITY_GAIN,
			D_VELOCITY_GAIN,
			F_VELOCITY_GAIN,
			this.batteryVoltageSensor.getVoltage()
		);


		this.imuUtil = new imuUtil(
			HARDWARE_MAP.get(IMU.class, "imu")
		);

		this.imuUtil.initialize(
			RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
			RevHubOrientationOnRobot.UsbFacingDirection.UP
		);


		this.pid = new pid(
			P_HEADING_GAIN,
			I_HEADING_GAIN,
			D_HEADING_GAIN,
			PID_MAX_SPEED
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

		double x = GAMEPAD.left_stick_x;
		double y = -GAMEPAD.left_stick_y;
		double turn = GAMEPAD.right_stick_x;

		double angle = Math.atan2(y, x);
		double power = Math.hypot(x, y);

		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);
		double max = Math.max(Math.abs(sin), Math.abs(cos));

		this.rightRearPower = (power * cos / max - turn);
		this.leftRearPower = (power * sin / max + turn);
		this.rightFrontPower = (power * sin / max - turn);
		this.leftFrontPower = (power * cos / max + turn);

		if (turn == 0 && this.lastTurn != turn) {
			this.holdHeading = true;
			this.targetHeading = -this.imuUtil.getHeading(AngleUnit.RADIANS);
			this.pid.reset();
		} else if (turn != 0) this.holdHeading = false;
		this.lastTurn = turn;

		if (this.holdHeading)
			this.fix = this.pid.headingController(Math.toDegrees(-this.targetHeading), this.imuUtil.getHeading());
		else this.fix = 0d;

		this.rightRearPower += this.fix;
		this.leftRearPower -= this.fix;
		this.rightFrontPower += this.fix;
		this.leftFrontPower -= this.fix;

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
		double x = GAMEPAD.left_stick_x;
		double y = -GAMEPAD.left_stick_y;
		double turn = GAMEPAD.right_stick_x;

		double heading = -this.imuUtil.getHeading(AngleUnit.RADIANS);

		if (GAMEPAD.a) this.imuUtil.reset();

		if (turn == 0 && this.lastTurn != turn) {
			this.holdHeading = true;
			this.targetHeading = -this.imuUtil.getHeading(AngleUnit.RADIANS);
			this.pid.reset();
		} else if (turn != 0) this.holdHeading = false;
		this.lastTurn = turn;

		if (this.holdHeading)
			this.fix = this.pid.headingController(Math.toDegrees(-this.targetHeading), this.imuUtil.getHeading());
		else this.fix = 0d;

		double rotX = x * Math.cos(heading) - y * Math.sin(heading);
		double rotY = x * Math.sin(heading) + y * Math.cos(heading);

		this.rightRearPower = rotY + rotX - turn;
		this.leftRearPower = rotY - rotX + turn;
		this.rightFrontPower = rotY - rotX - turn;
		this.leftFrontPower = rotY + rotX + turn;

		this.rightRearPower += this.fix;
		this.leftRearPower -= this.fix;
		this.rightFrontPower += this.fix;
		this.leftFrontPower -= this.fix;

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
		TELEMETRY.addData("Right rear: ", this.rightRearPower);
		TELEMETRY.addData("Left rear: ", this.leftRearPower);
		TELEMETRY.addData("Right front: ", this.rightFrontPower);
		TELEMETRY.addData("Left front: ", this.leftFrontPower);
		TELEMETRY.addData("Fix: ", this.fix);
		TELEMETRY.addLine();

		TELEMETRY.addData("Multiplier: ", this.speedMultiplier);
		TELEMETRY.addLine();

		TELEMETRY.addData("Hold heading: ", this.holdHeading);
		TELEMETRY.addData("Target heading: ", this.targetHeading);
		TELEMETRY.addLine();

		TELEMETRY.addLine();
	}
}
