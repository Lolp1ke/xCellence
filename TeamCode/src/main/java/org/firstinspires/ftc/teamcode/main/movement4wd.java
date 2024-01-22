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

	private DcMotor rightFront;
	private DcMotor leftFront;
	private DcMotor rightRear;
	private DcMotor leftRear;

	private Servo rocket;

	private IMU imu;

	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;
	private double speedMultiplier = 1d;

	private double rocketPosition = this._config.ROCKET_CLOSED;


	public void fieldCentric(final Gamepad gamepad) {
		boolean isBoosted = gamepad.right_bumper;
		boolean isSlowed = gamepad.left_bumper;
		this.speedMultiplier = isBoosted ? this._config.ACCELERATION : isSlowed ? this._config.DECELERATION : this._config.SPEED;

		double y = -gamepad.left_stick_y;
		double x = gamepad.left_stick_x;
		double rx = gamepad.right_stick_x;

		if (gamepad.a) {
			this.resetHeading();
		}

		double heading = -Math.toRadians(this.getHeading());

		double rotX = x * Math.cos(heading) - y * Math.sin(heading);
		double rotY = x * Math.sin(heading) + y * Math.cos(heading);

		double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
		this.rightRearPower = (rotY + rotX - rx) / denominator;
		this.leftRearPower = (rotY - rotX + rx) / denominator;
		this.rightFrontPower = (rotY - rotX - rx) / denominator;
		this.leftFrontPower = (rotY + rotX + rx) / denominator;

		this.rightRearPower *= this.speedMultiplier;
		this.leftRearPower *= this.speedMultiplier;
		this.rightFrontPower *= this.speedMultiplier;
		this.leftFrontPower *= this.speedMultiplier;

		this.rightRear.setPower(this.rightRearPower);
		this.leftRear.setPower(this.leftRearPower);
		this.rightFront.setPower(this.rightFrontPower);
		this.leftFront.setPower(this.leftFrontPower);
	}

	public void run(final Gamepad gamepad) {
		boolean isBoosted = gamepad.right_bumper;
		boolean isSlowed = gamepad.left_bumper;
		this.speedMultiplier = isBoosted ? this._config.ACCELERATION : isSlowed ? this._config.DECELERATION : this._config.SPEED;

		double x = gamepad.left_stick_x;
		double y = -gamepad.left_stick_y;
		double turn = gamepad.right_stick_x;

		double angle = Math.atan2(y, x);
		double power = Math.hypot(x, y);

		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);
		double max = Math.max(Math.abs(sin), Math.abs(cos));

		this.rightFrontPower = (power * sin / max - turn) * this.speedMultiplier;
		this.leftFrontPower = (power * cos / max + turn) * this.speedMultiplier;
		this.rightRearPower = (power * cos / max - turn) * this.speedMultiplier;
		this.leftRearPower = (power * sin / max + turn) * this.speedMultiplier;

		this.rightFrontPower = Range.clip(this.rightFrontPower, -1d, 1d);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -1d, 1d);
		this.rightRearPower = Range.clip(this.rightRearPower, -1d, 1d);
		this.leftRearPower = Range.clip(this.leftRearPower, -1d, 1d);


		if (gamepad.a && gamepad.x)
			this.rocketPosition = this._config.ROCKET_LAUNCHED;

		this.rocket.setPosition(this.rocketPosition);

		this.rightFront.setPower(this.rightFrontPower);
		this.leftFront.setPower(this.leftFrontPower);
		this.rightRear.setPower(this.rightRearPower);
		this.leftRear.setPower(this.leftRearPower);
	}


	private double getHeading() {
		return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	private void resetHeading() {
		this.imu.resetYaw();
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addLine("Movement");
		telemetry.addData("Speed multiplier: ", this.speedMultiplier);
		telemetry.addData("Heading: ", this.getHeading());
		telemetry.addData("Right front", this.rightFrontPower);
		telemetry.addData("Left front", this.leftFrontPower);
		telemetry.addData("Right rear", this.rightRearPower);
		telemetry.addData("Left rear", this.leftRearPower);
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

		this.rocket.setDirection(Servo.Direction.FORWARD);

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
