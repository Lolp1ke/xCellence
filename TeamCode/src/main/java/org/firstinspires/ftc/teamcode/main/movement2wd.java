package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

public class movement2wd {
	private final LinearOpMode opmode;
	private final config _config = new config();

	private DcMotor leftDrive;
	private DcMotor rightDrive;


	public movement2wd(LinearOpMode _opmode) {
		opmode = _opmode;
	}

	public void car() {
		boolean isBoosted = opmode.gamepad1.right_bumper;
		boolean isSlowed = opmode.gamepad1.left_bumper;
		double speedMultiplier = isBoosted ? _config.ACCELERATION : (isSlowed ? _config.DECELERATION : _config.SPEED);

		double drive = -opmode.gamepad1.left_stick_y;
		double turn = opmode.gamepad1.right_stick_x;

		double rightPower = Range.clip(drive - turn, -1.0, 1.0) * speedMultiplier;
		double leftPower = Range.clip(drive + turn, -1.0, 1.0) * speedMultiplier;

		rightDrive.setPower(rightPower);
		leftDrive.setPower(leftPower);

		opmode.telemetry.addData("Left: ", leftPower);
		opmode.telemetry.addData("Right: ", rightPower);
		opmode.telemetry.addData("Boosted?: ", isBoosted);
		opmode.telemetry.addData("Slowed?: ", isSlowed);
	}

	public void tank() {
		boolean isBoosted = opmode.gamepad1.right_bumper;
		boolean isSlowed = opmode.gamepad1.left_bumper;
		double speedMultiplier = isBoosted ? _config.ACCELERATION : (isSlowed ? _config.DECELERATION : _config.SPEED);

		double leftPower = -opmode.gamepad1.left_stick_y * speedMultiplier;
		double rightPower = opmode.gamepad1.right_stick_x * speedMultiplier;

		leftDrive.setPower(leftPower);
		rightDrive.setPower(rightPower);

		opmode.telemetry.addData("Left: ", leftPower);
		opmode.telemetry.addData("Right: ", rightPower);
		opmode.telemetry.addData("Boosted?: ", isBoosted);
		opmode.telemetry.addData("Slowed?: ", isSlowed);
	}

	public void init() {
		rightDrive = opmode.hardwareMap.get(DcMotor.class, "right_drive");
		leftDrive = opmode.hardwareMap.get(DcMotor.class, "left_drive");

		rightDrive.setDirection(DcMotor.Direction.FORWARD);
		leftDrive.setDirection(DcMotor.Direction.REVERSE);
	}
}
