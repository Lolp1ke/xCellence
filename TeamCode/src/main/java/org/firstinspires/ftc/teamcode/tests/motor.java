package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor test", group = "test")
public class motor extends LinearOpMode {
	private DcMotor rightFront;
	private DcMotor leftFront;
	private DcMotor rightRear;
	private DcMotor leftRear;

	private boolean isDpadUpPressed = false;
	private boolean isDpadDownPressed = false;

	public void runOpMode() {
		rightFront = hardwareMap.get(DcMotor.class, "right_front");
		leftFront = hardwareMap.get(DcMotor.class, "left_front");
		rightRear = hardwareMap.get(DcMotor.class, "right_rear");
		leftRear = hardwareMap.get(DcMotor.class, "left_rear");

		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		double speed = 0.3d;
		boolean dpadUp;
		boolean dpadDown = false;

		waitForStart();
		while (opModeIsActive()) {
			if ((dpadUp = gamepad1.dpad_up) && !isDpadUpPressed && speed < 1d) {
				speed += 0.1d;
			} else if ((dpadDown = gamepad1.dpad_down) && !isDpadDownPressed && speed > -1d) {
				speed -= 0.1d;
			}
			isDpadUpPressed = dpadUp;
			isDpadDownPressed = dpadDown;

			if (gamepad1.b) {
				rightFront.setPower(speed);
			} else {
				rightFront.setPower(0);
			}

			if (gamepad1.x) {
				leftFront.setPower(speed);
			} else {
				leftFront.setPower(0);
			}

			if (gamepad1.y) {
				rightRear.setPower(speed);
			} else {
				rightRear.setPower(0);
			}

			if (gamepad1.a) {
				leftRear.setPower(speed);
			} else {
				leftRear.setPower(0);
			}

			telemetry.addData("Speed: ", speed);
			telemetry.update();
		}
	}
}
