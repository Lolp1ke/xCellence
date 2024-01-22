package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "drive", group = "test")
public class ftclibMovement extends LinearOpMode {
	private Motor rightRear;
	private Motor leftRear;
	private Motor rightFront;
	private Motor leftFront;

	private MecanumDrive drive;
	private GamepadEx driverOp;

	private IMU imu;

	@Override
	public void runOpMode() {
		rightRear = new Motor(hardwareMap, "right_rear");
		leftRear = new Motor(hardwareMap, "left_rear");
		rightFront = new Motor(hardwareMap, "right_front");
		leftFront = new Motor(hardwareMap, "left_front");

//		rightRear = hardwareMap.get(Motor.class, "right_rear");
//		leftRear = hardwareMap.get(Motor.class, "left_rear");
//		rightFront = hardwareMap.get(Motor.class, "right_front");
//		leftFront = hardwareMap.get(Motor.class, "left_front");

		rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

		leftRear.setInverted(true);
		leftFront.setInverted(true);

		drive = new MecanumDrive(false, leftFront, rightFront, leftRear, rightRear);
		driverOp = new GamepadEx(gamepad1);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.initialize(new IMU.Parameters(
			new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
				RevHubOrientationOnRobot.UsbFacingDirection.UP
			)
		));
		imu.resetYaw();

		waitForStart();

		while (opModeIsActive()) {
			drive.driveRobotCentric(
				driverOp.getLeftX(),
				driverOp.getLeftY(),
				driverOp.getRightX(),
				true
			);

//			double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

//			drive.driveFieldCentric(driverOp.getLeftX(), driverOp.getLeftY(), driverOp.getRightX(), heading);
		}
	}
}