package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class movement4wd {
	private final config _config = new config();

	private DcMotor rightRear;
	private DcMotor leftRear;
	private DcMotor rightFront;
	private DcMotor leftFront;

	private Servo rocket;

	private IMU imu;

	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;

	private double speedMultiplier = _config.SPEED;

	private double headingError = 0d;
	private double lastError = 0d;
	private double totalError = 0d;
	private double fix = 0d;

	private double targetHeading = 0d;
	private boolean holdHeading = false;
	private boolean holdPressed = false;
	private boolean lastHoldPressed = false;

	private double lastRx = 0d;

	private double rocketPosition = this._config.ROCKET_CLOSED;


	public void fieldCentric(final Gamepad gamepad) {
//		boolean isBoosted = gamepad.right_bumper;
		boolean isSlowed = gamepad.left_bumper;
//		this.speedMultiplier = isBoosted ? this._config.ACCELERATION : isSlowed ? this._config.DECELERATION : this._config.SPEED;
		this.speedMultiplier = isSlowed ? this._config.DECELERATION : this._config.SPEED;

		double y = -gamepad.left_stick_y;
		double x = gamepad.left_stick_x;
		double rx = gamepad.right_stick_x;

		if (gamepad.a)
			this.resetHeading();

		double heading = -this.getHeading(AngleUnit.RADIANS);

		if (rx == 0 && this.lastRx != rx) {
			this.holdHeading = true;
			this.targetHeading = -this.getHeading(AngleUnit.RADIANS);
			this.PIDReset();
		} else if (rx != 0) {
			this.holdHeading = false;
		}
		this.lastRx = rx;

//		if ((this.holdPressed = gamepad.b) && !this.lastHoldPressed) {
//			this.targetHeading = -Math.toDegrees(heading);
//			this.holdHeading = !this.holdHeading;
//			this.PIDReset();
//		}
//		else this.holdHeading = false;

		if (this.holdHeading)
			this.fix = this.PIDControl(this.targetHeading, this.speedMultiplier,
				this._config.P_DRIVE_GAIN, this._config.I_DRIVE_GAIN, this._config.D_DRIVE_GAIN);
		else
			this.fix = 0d;

		double rotX = x * Math.cos(heading) - y * Math.sin(heading);
		double rotY = x * Math.sin(heading) + y * Math.cos(heading);

		this.rightRearPower = (rotY + rotX - rx);
		this.leftRearPower = (rotY - rotX + rx);
		this.rightFrontPower = (rotY - rotX - rx);
		this.leftFrontPower = (rotY + rotX + rx);

		this.rightRearPower += this.fix;
		this.leftRearPower -= this.fix;
		this.rightFrontPower += this.fix;
		this.leftFrontPower -= this.fix;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.speedMultiplier, this.speedMultiplier);

		this.rightRear.setPower(this.rightRearPower);
		this.leftRear.setPower(this.leftRearPower);
		this.rightFront.setPower(this.rightFrontPower);
		this.leftFront.setPower(this.leftFrontPower);
	}

	public void robotCentric(final Gamepad gamepad) {
//		boolean isBoosted = gamepad.right_bumper;
		boolean isSlowed = gamepad.left_bumper;
//		this.speedMultiplier = isBoosted ? this._config.ACCELERATION : isSlowed ? this._config.DECELERATION : this._config.SPEED;
		this.speedMultiplier = isSlowed ? this._config.DECELERATION : this._config.SPEED;

		double x = gamepad.left_stick_x;
		double y = -gamepad.left_stick_y;
		double turn = this.holdHeading ? 0d : gamepad.right_stick_x;

		double angle = Math.atan2(y, x);
		double power = Math.hypot(x, y);

		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);
		double max = Math.max(Math.abs(sin), Math.abs(cos));

		this.rightRearPower = (power * cos / max - turn);
		this.leftRearPower = (power * sin / max + turn);
		this.rightFrontPower = (power * sin / max - turn);
		this.leftFrontPower = (power * cos / max + turn);

		if ((this.holdPressed = gamepad.b) && !this.lastHoldPressed) {
			this.targetHeading = this.getHeading();
			this.holdHeading = !this.holdHeading;
			this.PIDReset();
		}
		this.lastHoldPressed = this.holdPressed;

		if (this.holdHeading)
			this.fix = this.PIDControl(this.targetHeading, this._config.SPEED,
				this._config.P_DRIVE_GAIN, this._config.I_DRIVE_GAIN, this._config.D_DRIVE_GAIN);
		else
			this.fix = 0d;

		this.rightRearPower += this.fix;
		this.leftRearPower -= this.fix;
		this.rightFrontPower += this.fix;
		this.leftFrontPower -= this.fix;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.speedMultiplier, this.speedMultiplier);

		if (gamepad.a && gamepad.x)
			this.rocketPosition = this._config.ROCKET_LAUNCHED;

		this.rocket.setPosition(this.rocketPosition);

		this.rightRear.setPower(this.rightRearPower);
		this.leftRear.setPower(this.leftRearPower);
		this.rightFront.setPower(this.rightFrontPower);
		this.leftFront.setPower(this.leftFrontPower);
	}

	private double PIDControl(final double HEADING, final double MAX_SPEED, final double P,
		final double I, final double D) {
		this.headingError = HEADING - this.getHeading();

		while (this.headingError > 180d) this.headingError -= 360d;
		while (this.headingError <= -180d) this.headingError += 360d;

		double finalError = this.headingError * P + this.totalError * I + (this.headingError - this.lastError) * D;

		this.lastError = this.headingError;
		this.totalError += this.headingError;
		return Range.clip(finalError, -MAX_SPEED, MAX_SPEED);
	}

	private void PIDReset() {
		this.headingError = 0d;
		this.lastError = 0d;
		this.totalError = 0d;
	}

	private double getHeading() {
		return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	private double getHeading(final AngleUnit angleUnit) {
		return this.imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
	}

	private void resetHeading() {
		this.imu.resetYaw();
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addLine("Movement");
		telemetry.addData("Speed multiplier: ", this.speedMultiplier);
		telemetry.addData("Right front", this.rightFrontPower);
		telemetry.addData("Left front", this.leftFrontPower);
		telemetry.addData("Right rear", this.rightRearPower);
		telemetry.addData("Left rear", this.leftRearPower);
		telemetry.addData("Fix: ", this.fix);
		telemetry.addLine();

		telemetry.addLine("Heading");
		telemetry.addData("Current: ", this.getHeading());
		telemetry.addData("Target: ", this.targetHeading);
		telemetry.addData("Hold: ", this.holdHeading);
		telemetry.addLine();

		telemetry.addLine("Error");
		telemetry.addData("Heading: ", this.headingError);
		telemetry.addData("Last: ", this.lastError);
		telemetry.addData("Total: ", this.totalError);
		telemetry.addLine();

		telemetry.addData("Rocket status: ", this.rocketPosition == this._config.ROCKET_LAUNCHED ? "Launched" : "Waiting");
		telemetry.addLine();
	}

	public void init(final HardwareMap hardwareMap) {
		this.rightFront = hardwareMap.get(DcMotor.class, "right_front");
		this.leftFront = hardwareMap.get(DcMotor.class, "left_front");
		this.rightRear = hardwareMap.get(DcMotor.class, "right_rear");
		this.leftRear = hardwareMap.get(DcMotor.class, "left_rear");

		this.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		this.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		this.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		this.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

		this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		this.rocket = hardwareMap.get(Servo.class, "rocket");

		this.rocket.setDirection(Servo.Direction.REVERSE);

		this.rocket.setPosition(this.rocketPosition);

		this.imu = hardwareMap.get(IMU.class, "imu");
		this.imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
					RevHubOrientationOnRobot.UsbFacingDirection.UP
				)
			)
		);
		this.imu.resetYaw();
	}
}
