package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class mechanism {
	private final LinearOpMode opMode;
	private final config _config = new config();
	private DcMotor rightArm;
	private DcMotor leftArm;
	private Servo hand;
	private Servo leftClaw;
	private Servo rightClaw;

	private ElapsedTime runtime = new ElapsedTime();

	public mechanism(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void placePurple() {
		hand.setPosition(_config.HAND_GROUND);
		opMode.sleep(1000);

		leftClaw.setPosition(_config.CLAW_OPENED + .05d);
		opMode.sleep(1000);

		hand.setPosition(_config.HAND_CLOSE);
		opMode.sleep(1000);
	}

	public void placeYellow() {
		encoderDrive(_config.ARM_SPEED, -17, -17, 3.0);
		opMode.sleep(1000);

		hand.setPosition(_config.HAND_CLOSE);
		opMode.sleep(1000);

		rightClaw.setPosition(_config.CLAW_OPENED);
		opMode.sleep(1000);
	}

	public void resetHand() {
		encoderDrive(1, 20, 20, 3);
	}

	public void encoderDrive(double speed,
		double leftInches,
		double rightInches,
		double timeoutS) {
		int newLeftTarget;
		int newRightTarget;

		if (opMode.opModeIsActive()) {
			newRightTarget = rightArm.getCurrentPosition() + (int) (rightInches * _config.ARM_COUNTS_PER_INCH);
			newLeftTarget = leftArm.getCurrentPosition() + (int) (leftInches * _config.ARM_COUNTS_PER_INCH);
			rightArm.setTargetPosition(newRightTarget);
			leftArm.setTargetPosition(newLeftTarget);

			rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			runtime.reset();
			rightArm.setPower(Math.abs(speed));
			leftArm.setPower(Math.abs(speed));

			while (opMode.opModeIsActive() &&
				(runtime.seconds() < timeoutS) &&
				(rightArm.isBusy() &&
					leftArm.isBusy())) {

				opMode.telemetry.addLine("Running to: ");
				opMode.telemetry.addData("Right: ", newRightTarget);
				opMode.telemetry.addData("Left: ", newLeftTarget);

				opMode.telemetry.addLine("Currently at: ");
				opMode.telemetry.addData("Right: ", rightArm.getCurrentPosition());
				opMode.telemetry.addData("Left: ", leftArm.getCurrentPosition());

				opMode.telemetry.update();
			}

			rightArm.setPower(0);
			leftArm.setPower(0);

			rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			opMode.sleep(250);
		}
	}

	public void init() {
		rightArm = opMode.hardwareMap.get(DcMotor.class, "right_arm");
		leftArm = opMode.hardwareMap.get(DcMotor.class, "left_arm");

		hand = opMode.hardwareMap.get(Servo.class, "hand");
		leftClaw = opMode.hardwareMap.get(Servo.class, "left_claw");
		rightClaw = opMode.hardwareMap.get(Servo.class, "right_claw");

		rightArm.setDirection(DcMotor.Direction.FORWARD);
		leftArm.setDirection(DcMotor.Direction.REVERSE);

		rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		hand.setDirection(Servo.Direction.REVERSE);
		rightClaw.setDirection(Servo.Direction.FORWARD);
		leftClaw.setDirection(Servo.Direction.REVERSE);

		rightClaw.setPosition(_config.CLAW_CLOSED);
		leftClaw.setPosition(_config.CLAW_CLOSED);
	}
}
