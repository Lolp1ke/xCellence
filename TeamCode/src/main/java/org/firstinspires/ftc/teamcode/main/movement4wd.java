package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class movement4wd {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private DcMotor rightFront;
	private DcMotor leftFront;
	private DcMotor rightRear;
	private DcMotor leftRear;

	private IMU imu;

	public movement4wd(final LinearOpMode _opMode) {
		opMode = _opMode;
	}


	public void test() {
		double x = opMode.gamepad1.left_stick_x;
		double y = -opMode.gamepad1.left_stick_y;
		double rx = opMode.gamepad1.right_stick_x;

		double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

		if (opMode.gamepad1.a)
			imu.resetYaw();

		double rotX = x * Math.cos(heading - Math.PI / 4) - y * Math.sin(heading - Math.PI / 4);
		double rotY = x * Math.sin(heading - Math.PI / 4) + y * Math.cos(heading - Math.PI / 4);

		double power = (rotY + rotX + rx) * 0.5d;
		double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
		double leftFrontPower = power / denominator;
		double leftRearPower = power / denominator;
		double rightFrontPower = power / denominator;
		double rightRearPower = power / denominator;

		rightFront.setPower(rightFrontPower);
		leftFront.setPower(leftFrontPower);
		rightRear.setPower(rightRearPower);
		leftRear.setPower(leftRearPower);

		opMode.telemetry.addData("Heading: ", heading);
		opMode.telemetry.addData("Right front", rightFrontPower);
		opMode.telemetry.addData("Left front", leftFrontPower);
		opMode.telemetry.addData("Right rear", rightRearPower);
		opMode.telemetry.addData("Left rear", leftRearPower);
	}

	public void run() {
		boolean isBoosted = opMode.gamepad1.right_bumper;
		boolean isSlowed = opMode.gamepad1.left_bumper;
		double speedMultiplier = isBoosted ? _config.ACCELERATION : isSlowed ? _config.DECELERATION : _config.SPEED;


		double x = opMode.gamepad1.left_stick_x;
		double y = -opMode.gamepad1.left_stick_y;
		double turn = opMode.gamepad1.right_stick_x;

		double angle = Math.atan2(y, x);
		double power = Math.hypot(x, y);

		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);
		double max = Math.max(Math.abs(sin), Math.abs(cos));

		double rightFrontPower = (power * sin / max - turn) * speedMultiplier;
		double leftFrontPower = (power * cos / max + turn) * speedMultiplier;
		double rightRearPower = (power * cos / max - turn) * speedMultiplier;
		double leftRearPower = (power * sin / max + turn) * speedMultiplier;

//		if ((power + Math.abs(turn)) > 1) {
//			rightFrontPower /= power + Math.abs(turn);
//			leftFrontPower /= power + Math.abs(turn);
//			rightRearPower /= power + Math.abs(turn);
//			leftRearPower /= power + Math.abs(turn);
//		}

		rightFrontPower = Range.clip(rightFrontPower, -1d, 1d);
		leftFrontPower = Range.clip(leftFrontPower, -1d, 1d);
		rightRearPower = Range.clip(rightRearPower, -1d, 1d);
		leftRearPower = Range.clip(leftRearPower, -1d, 1d);


		rightFront.setPower(rightFrontPower);
		leftFront.setPower(leftFrontPower);
		rightRear.setPower(rightRearPower);
		leftRear.setPower(leftRearPower);

		opMode.telemetry.addData("Angle: ", angle * 180d / 3.14d);
		opMode.telemetry.addData("Speed multiplier: ", speedMultiplier);
		opMode.telemetry.addData("Right front", rightFrontPower);
		opMode.telemetry.addData("Left front", leftFrontPower);
		opMode.telemetry.addData("Right rear", rightRearPower);
		opMode.telemetry.addData("Left rear", leftRearPower);
	}

	public void _run() {
		double normalizer;

		double x = opMode.gamepad1.left_stick_x;
		double y = -opMode.gamepad1.left_stick_y;
		double angle = opMode.gamepad1.right_stick_x;

		double rightFrontPower = (-x + y - angle) * _config.SPEED;
		double leftFrontPower = (x + y + angle) * _config.SPEED;
		double rightRearPower = (x + y - angle) * _config.SPEED;
		double leftRearPower = (-x + y + angle) * _config.SPEED;

		normalizer = Math.max(
			Math.max(Math.abs(rightFrontPower), Math.abs(leftFrontPower)),
			Math.max(Math.abs(rightRearPower), Math.abs(leftRearPower))
		);

		if (normalizer > 1) {
			rightFrontPower /= normalizer;
			leftFrontPower /= normalizer;
			rightRearPower /= normalizer;
			leftRearPower /= normalizer;
		}

		rightFront.setPower(rightFrontPower);
		leftFront.setPower(leftFrontPower);
		rightRear.setPower(rightRearPower);
		leftRear.setPower(leftRearPower);

		opMode.telemetry.addData("Right front", rightFrontPower);
		opMode.telemetry.addData("Left front", leftFrontPower);
		opMode.telemetry.addData("Right rear", rightRearPower);
		opMode.telemetry.addData("Left rear", leftRearPower);
	}

	public void init() {
		rightFront = opMode.hardwareMap.get(DcMotor.class, "right_front");
		leftFront = opMode.hardwareMap.get(DcMotor.class, "left_front");
		rightRear = opMode.hardwareMap.get(DcMotor.class, "right_rear");
		leftRear = opMode.hardwareMap.get(DcMotor.class, "left_rear");

//		rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//		leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//		rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
//		leftRear.setDirection(DcMotorSimple.Direction.FORWARD);


		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		imu = opMode.hardwareMap.get(IMU.class, "imu");
		imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
					RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
				)
			)
		);
		imu.resetYaw();
	}
}
