package org.firstinspires.ftc.teamcode.beta;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Movement", group = "test")
public class movement extends LinearOpMode {
	private Motor leftFront;
	private Motor rightFront;
	private Motor leftRear;
	private Motor rightRear;

	private MecanumDrive drive;
	private GamepadEx gamepad;
	private IMU imu;

	private double speedMultiplier = 1d;
	private double heading = 0d;

	@Override
	public void runOpMode() {
		leftFront = new Motor(hardwareMap, "left_front");
		rightFront = new Motor(hardwareMap, "right_front");
		leftRear = new Motor(hardwareMap, "left_rear");
		rightRear = new Motor(hardwareMap, "right_rear");

		rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

//		rightRear.setRunMode(Motor.RunMode.VelocityControl);

		drive = new MecanumDrive(
			leftFront,
			rightFront,
			leftRear,
			rightRear
		);
		drive.setRightSideInverted(false);

		gamepad = new GamepadEx(gamepad1);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
					RevHubOrientationOnRobot.UsbFacingDirection.UP
				)
			)
		);
		imu.resetYaw();

		waitForStart();
		while (opModeIsActive()) {
			if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER))
				speedMultiplier = 0.2d;
			else speedMultiplier = 0.7d;

			heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

			drive.driveFieldCentric(
				gamepad.getLeftX() * speedMultiplier,
				gamepad.getLeftY() * speedMultiplier,
				gamepad.getRightX() * speedMultiplier,
				heading,
				false
			);
		}
	}
}
