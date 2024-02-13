package org.firstinspires.ftc.teamcode.omega;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class movement2wd {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private DcMotor rightDrive;
	private DcMotor leftDrive;
	private IMU imu;


	private double driveSpeed = 0d;
	private double turnSpeed = 0d;
	private double rightSpeed = 0d;
	private double leftSpeed = 0d;
	private int rightTarget = 0;
	private int leftTarget = 0;
	private double headingError = 0.0d;
	private double targetHeading = 0.0d;
	private double oldError = 0.0d;
	private double errorSum = 0.0d;


	public movement2wd(final LinearOpMode _opMode) {
		opMode = _opMode;
	}


	public void straight(final double driveSpeed, final double distance, final double heading) {
		if (!opMode.opModeIsActive()) return;

		int target = (int) (distance * _config.COUNTS_PER_CM);
		rightTarget = rightDrive.getCurrentPosition() + target;
		leftTarget = leftDrive.getCurrentPosition() + target;

		rightDrive.setTargetPosition(rightTarget);
		leftDrive.setTargetPosition(leftTarget);

		rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		move(Math.abs(driveSpeed), 0);
		while (opMode.opModeIsActive() && rightDrive.isBusy() && leftDrive.isBusy()) {
			turnSpeed = PIDControl(heading);

			if (distance < 0)
				turnSpeed *= -1.0d;

			move(driveSpeed, turnSpeed);

			sendTelemetry(true);
		}

		move(0, 0);
		leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		opMode.sleep(1000);
	}

	public void move(final double drive, final double turn) {
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

	private double PIDControl(final double heading) {
		targetHeading = heading;
		headingError = targetHeading - getHeading();

		while (headingError > 180) headingError -= 360;
		while (headingError <= -180) headingError += 360;

		oldError = headingError;
		errorSum += headingError;
		return Range.clip(headingError * _config.P_DRIVE_GAIN + errorSum * _config.I_DRIVE_GAIN + (headingError - oldError) * _config.D_DRIVE_GAIN, -1, 1);
	}

	private void sendTelemetry(final boolean straight) {
		if (straight) {
			opMode.telemetry.addData("Target right: ", rightTarget);
			opMode.telemetry.addData("Target left: ", leftTarget);

			opMode.telemetry.addData("Current right: ", rightDrive.getCurrentPosition());
			opMode.telemetry.addData("Current left: ", leftDrive.getCurrentPosition());
		}

		opMode.telemetry.addLine("Heading");
		opMode.telemetry.addData("Target: ", targetHeading);
		opMode.telemetry.addData("Current: ", getHeading());

		opMode.telemetry.addData("Error:", headingError);
		opMode.telemetry.addData("Old error:", oldError);
		opMode.telemetry.addData("Error sum:", errorSum);

		opMode.telemetry.addData("Speed right: ", rightSpeed);
		opMode.telemetry.addData("Speed left: ", leftSpeed);
	}

	private double getHeading() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	public void resetHeading() {
		imu.resetYaw();
	}

	public void init() {
		rightDrive = opMode.hardwareMap.get(DcMotor.class, "right_drive");
		leftDrive = opMode.hardwareMap.get(DcMotor.class, "left_drive");

		rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

		imu = opMode.hardwareMap.get(IMU.class, "imu");

		imu.initialize(new IMU.Parameters(
			new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
			)
		));

		imu.resetYaw();
	}
}
