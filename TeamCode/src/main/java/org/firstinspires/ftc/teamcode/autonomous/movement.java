package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class movement {
	private final LinearOpMode opmode;
	private final config _config = new config();

	private DcMotor rightDrive;
	private DcMotor leftDrive;
	private IMU imu;


	private double headingError = 0.0d;
	private double targetHeading = 0.0d;
	private double driveSpeed = 0.0d;
	private double turnSpeed = 0.0d;
	private double rightSpeed = 0.0d;
	private double leftSpeed = 0.0d;
	private int rightTarget = 0;
	private int leftTarget = 0;

	public movement(final LinearOpMode _opmode) {
		opmode = _opmode;
	}

	public void straight(final double driveSpeed, final double distance, final double heading) {
		if (!opmode.opModeIsActive()) return;

		int target = (int) (distance * _config.COUNTS_PER_CM);
		rightTarget = rightDrive.getCurrentPosition() + target;
		leftTarget = leftDrive.getCurrentPosition() + target;

		rightDrive.setTargetPosition(rightTarget);
		leftDrive.setTargetPosition(leftTarget);

		rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		moveRobot(Math.abs(driveSpeed), 0);
		while (opmode.opModeIsActive() && rightDrive.isBusy() && leftDrive.isBusy()) {
			turnSpeed = proportionalController(heading, _config.P_DRIVE_GAIN);

			if (distance < 0)
				turnSpeed *= -1.0d;

			moveRobot(driveSpeed, turnSpeed);

			sendTelemetry(true);
		}

		moveRobot(0, 0);
		leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		opmode.sleep(1000);
	}

	public void turn(final double maxTurnSpeed, final double heading) {
		proportionalController(heading, _config.P_DRIVE_GAIN);

		while (opmode.opModeIsActive() && (Math.abs(headingError) > _config.HEADING_THRESHOLD)) {
			turnSpeed = proportionalController(heading, _config.P_TURN_GAIN);

			turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

			moveRobot(0, turnSpeed);

			sendTelemetry(false);
		}

		moveRobot(0, 0);
	}

	public void turnFix(double maxTurnSpeed, double heading, double holdTime) {
		ElapsedTime holdTimer = new ElapsedTime();
		holdTimer.reset();

		while (opmode.opModeIsActive() && (holdTimer.time() < holdTime)) {
			turnSpeed = proportionalController(heading, _config.P_TURN_GAIN);

			turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

			moveRobot(0, turnSpeed);

			sendTelemetry(false);
		}

		moveRobot(0, 0);
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


	public void moveRobot(final double drive, final double turn) {
		driveSpeed = drive;
		turnSpeed = turn;

		leftSpeed = drive - turn;
		rightSpeed = drive + turn;

		double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
		if (max > 1.0) {
			leftSpeed /= max;
			rightSpeed /= max;
		}

		leftDrive.setPower(leftSpeed);
		rightDrive.setPower(rightSpeed);
	}

	public void sendTelemetry(final boolean straight) {
		if (straight) {
			opmode.telemetry.addData("Target right: ", rightTarget);
			opmode.telemetry.addData("Target left: ", leftTarget);

			opmode.telemetry.addData("Current right: ", rightDrive.getCurrentPosition());
			opmode.telemetry.addData("Current left: ", leftDrive.getCurrentPosition());
		}

		opmode.telemetry.addLine("Heading");
		opmode.telemetry.addData("Target: ", targetHeading);
		opmode.telemetry.addData("Current: ", getHeading());

		opmode.telemetry.addData("Error:", headingError);
		opmode.telemetry.addData("Steer:", turnSpeed);

		opmode.telemetry.addData("Speed right: ", rightSpeed);
		opmode.telemetry.addData("Speed left: ", leftSpeed);

		opmode.telemetry.update();
	}

	public void init() {
		rightDrive = opmode.hardwareMap.get(DcMotor.class, "right_drive");
		leftDrive = opmode.hardwareMap.get(DcMotor.class, "left_drive");

		rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

		rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		imu = opmode.hardwareMap.get(IMU.class, "imu");

		imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.UP,
					RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
				)
			)
		);

		imu.resetYaw();
	}
}
