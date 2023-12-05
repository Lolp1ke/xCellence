package org.firstinspires.ftc.teamcode.f1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "F1", group = "!!!F1")
@Disabled
public class f1 extends LinearOpMode {
	private final config _config = new config();

	private DcMotor rightMotor;
	private DcMotor leftMotor;

	private IMU imu;

	private double inputHeading = 0.0d;

	@Override
	public void runOpMode() {
		startEngine();
		telemetry.addLine("MCL60 is ready to drive");
		telemetry.update();

		waitForStart();
		if (opModeIsActive()) {
			telemetry.clearAll();

			while (opModeIsActive()) {
				run();

				telemetry.update();
			}
		}
	}

	private void run() {
		boolean isBoosted = gamepad1.right_bumper;
		boolean isSlowed = gamepad1.right_bumper;
		double multiplier = isBoosted ? _config.ACCELERATION
			: (isSlowed ? _config.DECELERATION : _config.SPEED);

		double drive = -gamepad1.left_stick_y;
		double turn = gamepad1.right_stick_x;

		if (drive != 0)
			inputHeading = Math.atan(turn / drive);

		double rightPower = (drive - turn) * multiplier;
		double leftPower = (drive + turn) * multiplier;

		double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

		rightMotor.setPower(rightPower);
		leftMotor.setPower(leftPower);

		telemetry.addLine("Motor power");
		telemetry.addData("Right: ", rightPower);
		telemetry.addData("Left: ", leftPower);
		telemetry.addLine();

		telemetry.addLine("Heading");
		telemetry.addData("Current: ", currentHeading);
		telemetry.addData("Input: ", inputHeading);
	}

	private void startEngine() {
		rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
		leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");

		rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
		leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		imu = hardwareMap.get(IMU.class, "imu");

		imu.initialize(new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.UP,
					RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
				)
			)
		);

		imu.resetYaw();
	}
}