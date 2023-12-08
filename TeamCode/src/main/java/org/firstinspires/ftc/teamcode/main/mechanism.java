package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class mechanism {
	private final LinearOpMode opmode;
	private final config _config = new config();

	private DcMotor rightArm;
	private DcMotor leftArm;

	private DcMotor lift;

	private Servo hand;

	private Servo rightClaw;
	private Servo leftClaw;

	private Servo rocket;

	private double handPosition = _config.HAND_CLOSE;
	private double rocketPosition = _config.ROCKET_CLOSED;
	private boolean hang = false;

	public mechanism(final LinearOpMode _opMode) {
		opmode = _opMode;
	}

	public void run() {
		double armPower = opmode.gamepad2.left_stick_y * (opmode.gamepad2.left_trigger != 0 ? _config.ARM_BOOST : _config.ARM_SPEED);
		double liftPower = opmode.gamepad2.right_stick_y * (opmode.gamepad2.right_trigger != 0 ? _config.LIFT_BOOST : _config.LIFT_SPEED);

		if (opmode.gamepad2.x) {
			handPosition = _config.HAND_CLOSE;
		} else if (opmode.gamepad2.a) {
			handPosition = _config.HAND_GROUND;
		} else if (opmode.gamepad2.b) {
			handPosition = _config.HAND_MID;
		}

		if (opmode.gamepad2.right_bumper) {
			armPower = -0.25d;
		} else if (opmode.gamepad2.left_bumper) {
			armPower = -0.1d;
		}

		if (opmode.gamepad2.dpad_up) {
			hang = true;
		} else if (opmode.gamepad2.dpad_down) {
			hang = false;
		}

		if (opmode.gamepad2.dpad_right && opmode.gamepad2.a) {
			rocketPosition = _config.ROCKET_LAUNCHED;
		} else if (opmode.gamepad2.dpad_right && opmode.gamepad2.b) {
			rocketPosition = _config.ROCKET_LAUNCHED;
		}

		rightArm.setPower(hang ? 0.85d : armPower);
		leftArm.setPower(hang ? 0.85d : armPower);
		lift.setPower(liftPower);

		hand.setPosition(handPosition);
		rightClaw.setPosition(opmode.gamepad2.y ? _config.CLAW_CLOSE : opmode.gamepad2.dpad_right ? _config.CLAW_CLOSE : _config.CLAW_OPEN);
		leftClaw.setPosition(opmode.gamepad2.y ? _config.CLAW_CLOSE : opmode.gamepad2.dpad_left ? _config.CLAW_CLOSE : _config.CLAW_OPEN);

		rocket.setPosition(rocketPosition);

		opmode.telemetry.addData("Arm: ", armPower);
		opmode.telemetry.addData("Hand: ", handPosition);
		opmode.telemetry.addData("Claw: ", opmode.gamepad2.y);
		opmode.telemetry.addData("Right claw: ", opmode.gamepad2.dpad_up);
		opmode.telemetry.addData("Left claw: ", opmode.gamepad2.dpad_down);
		opmode.telemetry.addData("Rocket: ", rocketPosition);
	}

	public void init() {
		rightArm = opmode.hardwareMap.get(DcMotor.class, "right_arm");
		leftArm = opmode.hardwareMap.get(DcMotor.class, "left_arm");
		lift = opmode.hardwareMap.get(DcMotor.class, "lift");

		hand = opmode.hardwareMap.get(Servo.class, "hand");
		rightClaw = opmode.hardwareMap.get(Servo.class, "right_claw");
		leftClaw = opmode.hardwareMap.get(Servo.class, "left_claw");

		rocket = opmode.hardwareMap.get(Servo.class, "rocket");

		rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
		leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
		lift.setDirection(DcMotorSimple.Direction.FORWARD);

		hand.setDirection(Servo.Direction.REVERSE);
		rightClaw.setDirection(Servo.Direction.FORWARD);
		leftClaw.setDirection(Servo.Direction.REVERSE);

		rocket.setDirection(Servo.Direction.FORWARD);

		hand.setPosition(handPosition);
		rightClaw.setPosition(_config.CLAW_OPEN);
		leftClaw.setPosition(_config.CLAW_OPEN);

		rocket.setPosition(_config.ROCKET_CLOSED);
	}
}
