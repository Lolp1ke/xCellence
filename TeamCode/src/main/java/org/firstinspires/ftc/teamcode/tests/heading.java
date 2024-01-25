package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.autonomous.config;
import org.firstinspires.ftc.teamcode.utils.motorUtil;

@TeleOp(name = "Heading keeper", group = "test")
public class heading extends LinearOpMode {
	private final config _config = new config();
	private DcMotorEx rightRear;
	private DcMotorEx leftRear;
	private DcMotorEx rightFront;
	private DcMotorEx leftFront;

	private motorUtil motors;

	private IMU imu;

	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;

	private double speedMultiplier = 1d;

	private double headingError = 0d;
	private double lastError = 0d;
	private double totalError = 0d;
	private double fix = 0d;

	private double targetHeading = 0d;

	private boolean holdPressed = false;
	private boolean lastHoldPressed = false;
	private boolean holdHeading = false;

	@Override
	public void runOpMode() {
		this.motors = new motorUtil(this.rightRear, this.leftRear, this.rightFront, this.leftFront);
		this.motors.init(hardwareMap,
			"right_rear", "left_rear",
			"right_front", "left_front");

		this.motors.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

		this.motors.setDirection(
			DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
			DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE
		);

		this.imu = hardwareMap.get(IMU.class, "imu");
		this.imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
					RevHubOrientationOnRobot.UsbFacingDirection.UP
				)
			)
		);
		this.resetHeading();


		waitForStart();
		while (opModeIsActive()) {
			boolean isBoosted = gamepad1.right_bumper;
			boolean isSlowed = gamepad1.left_bumper;
			this.speedMultiplier = isBoosted ? 0.7d : isSlowed ? 0.3d : 0.5d;

			double x = gamepad1.left_stick_x;
			double y = -gamepad1.left_stick_y;
			double turn = gamepad1.right_stick_x;

			double angle = Math.atan2(y, x);
			double power = Math.hypot(x, y);

			double sin = Math.sin(angle - Math.PI / 4);
			double cos = Math.cos(angle - Math.PI / 4);
			double max = Math.max(Math.abs(sin), Math.abs(cos));

			this.rightFrontPower = (power * sin / max - turn) * this.speedMultiplier;
			this.leftFrontPower = (power * cos / max + turn) * this.speedMultiplier;
			this.rightRearPower = (power * cos / max - turn) * this.speedMultiplier;
			this.leftRearPower = (power * sin / max + turn) * this.speedMultiplier;

			if ((this.holdPressed = gamepad1.b) && !this.lastHoldPressed) {
				this.targetHeading = this.getHeading();
				this.holdHeading = !this.holdHeading;
			}
			this.lastHoldPressed = this.holdPressed;

			if (this.holdHeading)
				this.fix = this.PIDControl(this.targetHeading, _config.SPEED, _config.P_DRIVE_GAIN, _config.I_DRIVE_GAIN, _config.D_DRIVE_GAIN);
			else this.fix = 0d;

			this.rightRearPower += this.fix;
			this.leftRearPower -= this.fix;
			this.rightFrontPower += this.fix;
			this.leftFrontPower -= this.fix;

			this.rightFrontPower = Range.clip(this.rightFrontPower, -1d, 1d);
			this.leftFrontPower = Range.clip(this.leftFrontPower, -1d, 1d);
			this.rightRearPower = Range.clip(this.rightRearPower, -1d, 1d);
			this.leftRearPower = Range.clip(this.leftRearPower, -1d, 1d);

			this.motors.setPower(
				this.rightRearPower, this.leftRearPower,
				this.rightFrontPower, this.leftFrontPower
			);

			telemetry.addLine("Heading");
			telemetry.addData("Current: ", this.getHeading());
			telemetry.addData("Target", this.targetHeading);
			telemetry.addData("Heading error: ", this.headingError);
			telemetry.addData("Last error: ", this.lastError);
			telemetry.addData("Total error: ", this.totalError);
			telemetry.addLine();

			telemetry.addLine("Power");
			telemetry.addData("Right rear: ", this.rightRearPower);
			telemetry.addData("Left rear", this.leftRearPower);
			telemetry.addData("Right front", this.rightFrontPower);
			telemetry.addData("Left front", this.leftFrontPower);
			telemetry.addData("Turn: ", this.fix);
			telemetry.addLine();

			telemetry.update();
		}
	}

	private double PIDControl(final double HEADING, final double MAX_SPEED, final double P,
		final double I, final double D) {
		this.headingError = HEADING - getHeading();

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

	public void resetHeading() {
		this.imu.resetYaw();
	}
}
