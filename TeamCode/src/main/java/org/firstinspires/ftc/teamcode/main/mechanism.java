package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class mechanism {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private DcMotor rightArm;
	private DcMotor leftArm;
	private DcMotor lift;

	private Servo hand;
	private Servo rightClaw;
	private Servo leftClaw;

	private Servo rocket;

	private double handPosition = _config.HAND_SCORE;
	private double rightClawPosition = _config.CLAW_OPEN;
	private double leftClawPosition = _config.CLAW_OPEN;

	private double rocketPosition = _config.ROCKET_CLOSED;

//	private boolean hang = false;


	public mechanism(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void run() {
		double armPower = opMode.gamepad2.left_stick_y *
			(opMode.gamepad2.left_trigger >= 0.4d ? _config.ARM_BOOST : _config.ARM_SPEED);
		double liftPower = opMode.gamepad2.right_stick_y * _config.LIFT_SPEED;
		int armPosition = (rightArm.getCurrentPosition() + leftArm.getCurrentPosition()) / 2;
		int liftPosition = lift.getCurrentPosition();

//		if (Math.abs(armPower) == 0d && lastArmPower != armPower) {
//			rightArm.setTargetPosition(armPosition);
//			leftArm.setTargetPosition(armPosition);
//
//			rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//			rightArm.setPower(1d);
//			leftArm.setPower(1d);
//		} else {
//			rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		}


//		double handOffset = liftPosition / 10000d;
		if (opMode.gamepad2.x) handPosition = _config.HAND_SCORE;
		else if (opMode.gamepad2.a) handPosition = _config.HAND_GROUND; // + handOffset;
		else if (opMode.gamepad2.b) handPosition = _config.HAND_MID;


		if (opMode.gamepad2.right_trigger > 0.4d) {
			rightClawPosition = _config.CLAW_CLOSE;
			leftClawPosition = _config.CLAW_CLOSE;
		} else if (opMode.gamepad2.dpad_right)
			rightClawPosition = _config.CLAW_CLOSE;
		else if (opMode.gamepad2.dpad_left)
			leftClawPosition = _config.CLAW_CLOSE;
		else {
			rightClawPosition = _config.CLAW_OPEN;
			leftClawPosition = _config.CLAW_OPEN;
		}


		if (opMode.gamepad1.a && opMode.gamepad1.x)
			rocketPosition = _config.ROCKET_LAUNCHED;


//		if (opMode.gamepad2.dpad_up) hang = true;
//		else if (opMode.gamepad2.dpad_down) hang = false;

		rightArm.setPower(armPower);
		leftArm.setPower(armPower);
		lift.setPower(liftPower);

		//		lift.setPower(hang ? 0.0566d : liftPower);

		hand.setPosition(handPosition);
		rightClaw.setPosition(rightClawPosition);
		leftClaw.setPosition(leftClawPosition);
		rocket.setPosition(rocketPosition);

		opMode.telemetry.addLine("Mechanism");
		opMode.telemetry.addData("Arm: ", armPower);
		opMode.telemetry.addData("Arm position: ", armPosition);
		opMode.telemetry.addData("Lift: ", liftPower);
		opMode.telemetry.addData("Lift position: ", liftPosition);
		opMode.telemetry.addData("Hand: ", handPosition);
		opMode.telemetry.addData("Right claw: ", rightClawPosition);
		opMode.telemetry.addData("Left claw: ", leftClawPosition);
//		opMode.telemetry.addData("Hand offset: ", handOffset);
		opMode.telemetry.addData("Rocket: ", rocketPosition);
		opMode.telemetry.addLine();
	}

	public void init() {
		rightArm = opMode.hardwareMap.get(DcMotor.class, "right_arm");
		leftArm = opMode.hardwareMap.get(DcMotor.class, "left_arm");
		lift = opMode.hardwareMap.get(DcMotor.class, "lift");

		hand = opMode.hardwareMap.get(Servo.class, "hand");
		rightClaw = opMode.hardwareMap.get(Servo.class, "right_claw");
		leftClaw = opMode.hardwareMap.get(Servo.class, "left_claw");

		rocket = opMode.hardwareMap.get(Servo.class, "rocket");

		rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
		leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
		lift.setDirection(DcMotorSimple.Direction.FORWARD);

		rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		hand.setDirection(Servo.Direction.FORWARD);
		rightClaw.setDirection(Servo.Direction.FORWARD);
		leftClaw.setDirection(Servo.Direction.REVERSE);

		rocket.setDirection(Servo.Direction.FORWARD);

		hand.setPosition(handPosition);
		rightClaw.setPosition(leftClawPosition);
		leftClaw.setPosition(rightClawPosition);

		rocket.setPosition(rocketPosition);
	}
}
