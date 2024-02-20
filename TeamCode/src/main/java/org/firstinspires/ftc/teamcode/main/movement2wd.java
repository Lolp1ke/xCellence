package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class movement2wd {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private DcMotor rightDrive;
	private DcMotor leftDrive;

	private boolean driveModeToggle = false;
	private boolean lastDriveModeToggle = false;


	public movement2wd(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void car() {
		boolean isBoosted = opMode.gamepad1.right_bumper;
		boolean isSlowed = opMode.gamepad1.left_bumper;

		double drive = -opMode.gamepad1.left_stick_y;
		double turn = opMode.gamepad1.right_stick_x;

		double rightPower = Range.clip(drive - turn, -1.0, 1.0);
		double leftPower = Range.clip(drive + turn, -1.0, 1.0);

		double speedMultiplier = isBoosted ? (Math.abs(turn) > 0.2d ? config.SPEED : config.ACCELERATION) : (isSlowed ? config.DECELERATION : config.SPEED);
		rightDrive.setPower(rightPower * speedMultiplier);
		leftDrive.setPower(leftPower * speedMultiplier);

		if ((driveModeToggle = opMode.gamepad1.y) && !lastDriveModeToggle) {
			rightDrive.setZeroPowerBehavior(rightDrive.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
			leftDrive.setZeroPowerBehavior(leftDrive.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE);
		}
		lastDriveModeToggle = driveModeToggle;

		opMode.telemetry.addLine("Movement");
		opMode.telemetry.addData("Left: ", leftPower);
		opMode.telemetry.addData("Right: ", rightPower);
		opMode.telemetry.addData("Speed multiplier: ", speedMultiplier);
		opMode.telemetry.addData("Drive mode: ", rightDrive.getZeroPowerBehavior());
		opMode.telemetry.addLine();
	}

	public void init() {
		rightDrive = opMode.hardwareMap.get(DcMotor.class, "right_drive");
		leftDrive = opMode.hardwareMap.get(DcMotor.class, "left_drive");

		rightDrive.setDirection(DcMotor.Direction.FORWARD);
		leftDrive.setDirection(DcMotor.Direction.REVERSE);

		rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
	}
}
