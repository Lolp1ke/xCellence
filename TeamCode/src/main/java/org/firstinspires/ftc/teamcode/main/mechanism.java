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
	private boolean hang = false;

	public mechanism(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void run() {
		double armPower = opMode.gamepad2.left_stick_y *
			(opMode.gamepad2.left_trigger != 0 ? _config.ARM_BOOST : _config.ARM_SPEED);
		double liftPower = opMode.gamepad2.right_stick_y * _config.LIFT_SPEED;

		if (opMode.gamepad2.x) handPosition = _config.HAND_SCORE;
		else if (opMode.gamepad2.a) handPosition = _config.HAND_GROUND;
		else if (opMode.gamepad2.b) handPosition = _config.HAND_MID;


		if (opMode.gamepad2.left_bumper) armPower = -0.1d;


		if (opMode.gamepad2.dpad_up) hang = true;
		else if (opMode.gamepad2.dpad_down) hang = false;


		if (opMode.gamepad2.dpad_right && opMode.gamepad2.a)
			rocketPosition = _config.ROCKET_LAUNCHED;
		else if (opMode.gamepad2.dpad_right && opMode.gamepad2.b)
			rocketPosition = _config.ROCKET_CLOSED;

		if (opMode.gamepad2.right_trigger > 0.4d) {
			rightClawPosition = _config.CLAW_CLOSE;
			leftClawPosition = _config.CLAW_CLOSE;
		} else if (opMode.gamepad2.dpad_right)
			rightClawPosition = _config.CLAW_CLOSE;
		else if (opMode.gamepad2.dpad_left)
			leftClawPosition = _config.CLAW_CLOSE;
		else {
			rightClawPosition = _config.CLAW_CLOSE;
			leftClawPosition = _config.CLAW_CLOSE;
		}

		rightArm.setPower(hang ? -0.2d : armPower);
		leftArm.setPower(hang ? -0.2d : armPower);
		lift.setPower(hang ? 0.2d : liftPower);

		hand.setPosition(handPosition);
		rightClaw.setPosition(rightClawPosition);
		leftClaw.setPosition(leftClawPosition);
		rocket.setPosition(rocketPosition);

		opMode.telemetry.addLine("Mechanism");
		opMode.telemetry.addData("Arm: ", armPower);
		opMode.telemetry.addData("Hand: ", handPosition);
		opMode.telemetry.addData("Right claw: ", rightClawPosition);
		opMode.telemetry.addData("Left claw: ", leftClawPosition);
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

		hand.setDirection(Servo.Direction.REVERSE);
		rightClaw.setDirection(Servo.Direction.FORWARD);
		leftClaw.setDirection(Servo.Direction.REVERSE);

		rocket.setDirection(Servo.Direction.FORWARD);

		hand.setPosition(handPosition);
		rightClaw.setPosition(leftClawPosition);
		leftClaw.setPosition(rightClawPosition);

		rocket.setPosition(rocketPosition);
	}
}
