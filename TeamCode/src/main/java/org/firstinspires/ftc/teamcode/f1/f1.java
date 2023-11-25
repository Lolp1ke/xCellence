package org.firstinspires.ftc.teamcode.f1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "F1", group = "!!!F1")
public class f1 extends LinearOpMode {
	private final config _config = new config();

	private DcMotor rightMotor;
	private DcMotor leftMotor;
	private IMU imu;

	private double turnSpeed = 0.0d;
	private double heading = 0.0d;
	private double targetHeading = 0.0d;
	private double headingError = 0.0d;

	@Override
	public void runOpMode() {
		startEngine();

		waitForStart();
		while (opModeIsActive()) {
			final double drive = gamepad1.left_stick_y;
			double turn = gamepad1.right_stick_x;
			if (drive + turn == 0) {
				rightMotor.setPower(0);
				leftMotor.setPower(0);

				rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				continue;
			}

			final boolean isBoosted = gamepad1.right_bumper;
			final boolean isSlowed = gamepad1.left_bumper;
			final double multiplier = isBoosted ? _config.ACCELERATION : (isSlowed ? _config.DECELERATION : _config.SPEED);

			if (drive != 0)
				heading = Math.atan(turn / drive) * (drive / Math.abs(drive));
			turn = proportionalController(heading, _config.P_DRIVE_GAIN);

			final double rightPower = leftMotor.getCurrentPosition() + Range.clip(drive - turn, -1.0d, 1.0d) * 1000;
			final double leftPower = rightMotor.getCurrentPosition() + Range.clip(drive + turn, -1.0d, 1.0d) * 1000;

			rightMotor.setTargetPosition((int) rightPower);
			leftMotor.setTargetPosition((int) leftPower);

			rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			rightMotor.setPower(multiplier);
			leftMotor.setPower(multiplier);
		}
	}

	public double proportionalController(final double desiredHeading, final double proportionalGain) {
		targetHeading = desiredHeading;

		headingError = targetHeading - getHeading();

		while (headingError > 180) headingError -= 360;
		while (headingError <= -180) headingError += 360;

		return Range.clip(headingError * proportionalGain, -1, 1);
	}

	public double getHeading() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	private void startEngine() {
		rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
		leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");

		rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.UP,
					RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
				)
			)
		);
		imu.resetYaw();

		telemetry.addData("Engine status: ", "Initialized");
		telemetry.update();
	}
}