package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class mechanism {
	private final LinearOpMode opmode;
	private DcMotor rightArm;
	private DcMotor leftArm;
	private Servo hand;
	private Servo leftClaw;
	private Servo rightClaw;

	private ElapsedTime runtime = new ElapsedTime();

	private final double COUNTS_PER_MOTOR_REV = 288;
	private final double DRIVE_GEAR_REDUCTION = 1.0;
	private final double WHEEL_DIAMETER_INCHES = 9.5 / 2.54;
	private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
		(WHEEL_DIAMETER_INCHES * 3.1415);
	private final double DRIVE_SPEED = 0.6;
	private final double TURN_SPEED = 0.5;

	private final double HAND_GROUND = 0.0d;
	private final double HAND_CLOSE = 1.0d;

	private final double CLAW_OPENED = 0.1d;
	private final double CLAW_CLOSED = 0.3d;

	public mechanism(final LinearOpMode _opmode) {
		opmode = _opmode;
	}

	public void init() {
		rightArm = opmode.hardwareMap.get(DcMotor.class, "right_arm");
		leftArm = opmode.hardwareMap.get(DcMotor.class, "left_arm");

		hand = opmode.hardwareMap.get(Servo.class, "hand");
		leftClaw = opmode.hardwareMap.get(Servo.class, "left_claw");
		rightClaw = opmode.hardwareMap.get(Servo.class, "right_claw");

		rightArm.setDirection(DcMotor.Direction.FORWARD);
		leftArm.setDirection(DcMotor.Direction.REVERSE);

		rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		hand.setDirection(Servo.Direction.REVERSE);
		rightClaw.setDirection(Servo.Direction.FORWARD);
		leftClaw.setDirection(Servo.Direction.REVERSE);

		rightClaw.setPosition(CLAW_CLOSED);
		leftClaw.setPosition(CLAW_CLOSED);
	}

	public void placePurple() {
		hand.setPosition(HAND_GROUND);
		opmode.sleep(1000);

		leftClaw.setPosition(CLAW_OPENED + .05d);
		opmode.sleep(1000);

		hand.setPosition(HAND_CLOSE);
		opmode.sleep(1000);
	}

	public void placeYellow() {
		encoderDrive(DRIVE_SPEED, -17, -17, 3.0);
		opmode.sleep(1000);

		hand.setPosition(HAND_CLOSE);
		opmode.sleep(1000);

		rightClaw.setPosition(CLAW_OPENED);
		opmode.sleep(1000);
	}

	public void resetHand() {
		encoderDrive(1, 20, 20, 3);
	}


	public void encoderDrive(double speed,
		double leftInches, double rightInches,
		double timeoutS) {
		int newLeftTarget;
		int newRightTarget;

		if (opmode.opModeIsActive()) {
			newRightTarget = rightArm.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
			newLeftTarget = leftArm.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
			rightArm.setTargetPosition(newRightTarget);
			leftArm.setTargetPosition(newLeftTarget);

			rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			runtime.reset();
			rightArm.setPower(Math.abs(speed));
			leftArm.setPower(Math.abs(speed));

			while (opmode.opModeIsActive() &&
				(runtime.seconds() < timeoutS) &&
				(rightArm.isBusy() && leftArm.isBusy())) {

				opmode.telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
				opmode.telemetry.addData("Currently at", " at %7d :%7d",
					leftArm.getCurrentPosition(), rightArm.getCurrentPosition());
				opmode.telemetry.update();
			}

			rightArm.setPower(0);
			leftArm.setPower(0);

			rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			opmode.sleep(250);
		}
	}
}
