package org.firstinspires.ftc.teamcode.alpha;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class mechanism {
	private final config _config = new config();

	private DcMotor rightArm;
	private DcMotor leftArm;
	private DcMotor lift;

	private Servo wrist;
	private Servo rightClaw;
	private Servo leftClaw;

	private double armPower = 0d;
	private double liftPower = 0d;

	private int armPosition = 0;
	private int liftPosition = 0;

	private double wristPosition = config.WRIST_SCORE;
	private double rightClawPosition = config.CLAW_CLOSE;
	private double leftClawPosition = config.CLAW_CLOSE;


	public void run(final Gamepad gamepad) {
		armPower = gamepad.left_stick_y *
			(gamepad.left_trigger >= 0.4d ? config.ARM_BOOST : config.ARM_SPEED);
		liftPower = gamepad.right_stick_y * config.LIFT_SPEED;
		armPosition = (this.rightArm.getCurrentPosition() + this.leftArm.getCurrentPosition()) / 2;
		liftPosition = this.lift.getCurrentPosition();

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
		if (gamepad.x) this.wristPosition = config.WRIST_SCORE;
		else if (gamepad.a) this.wristPosition = config.WRIST_GROUND; // + handOffset;
		else if (gamepad.b) this.wristPosition = config.WRIST_MID;

		this.rightClawPosition = config.CLAW_OPEN;
		this.leftClawPosition = config.CLAW_OPEN;

		if (gamepad.right_trigger > 0.4d) {
			this.rightClawPosition = config.CLAW_CLOSE;
			this.leftClawPosition = config.CLAW_CLOSE;
		}

		if (gamepad.dpad_right)
			this.rightClawPosition = config.CLAW_CLOSE;
		else if (gamepad.dpad_left)
			this.leftClawPosition = config.CLAW_CLOSE;
//		else {
//
//		}

//		if (gamepad.dpad_up) hang = true;
//		else if (gamepad.dpad_down) hang = false;

		this.rightArm.setPower(this.armPower);
		this.leftArm.setPower(this.armPower);
		this.lift.setPower(this.liftPower);

		//		lift.setPower(hang ? 0.0566d : liftPower);

		this.wrist.setPosition(this.wristPosition);
		this.rightClaw.setPosition(this.rightClawPosition);
		this.leftClaw.setPosition(this.leftClawPosition);
	}

	public void telemetry(Telemetry telemetry) {
		telemetry.addLine("Mechanism");
		telemetry.addData("Arm: ", this.armPower);
		telemetry.addData("Arm position: ", this.armPosition);
		telemetry.addData("Lift: ", this.liftPower);
		telemetry.addData("Lift position: ", this.liftPosition);
		telemetry.addData("Wrist: ", this.wristPosition);
		telemetry.addData("Right claw: ", this.rightClawPosition);
		telemetry.addData("Left claw: ", this.leftClawPosition);
		telemetry.addLine();
	}

	public void init(final HardwareMap hardwareMap) {
		this.rightArm = hardwareMap.get(DcMotor.class, "right_arm");
		this.leftArm = hardwareMap.get(DcMotor.class, "left_arm");
		this.lift = hardwareMap.get(DcMotor.class, "lift");

		this.wrist = hardwareMap.get(Servo.class, "wrist");
		this.rightClaw = hardwareMap.get(Servo.class, "right_claw");
		this.leftClaw = hardwareMap.get(Servo.class, "left_claw");

		this.rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
		this.leftArm.setDirection(DcMotorSimple.Direction.REVERSE);
		this.lift.setDirection(DcMotorSimple.Direction.FORWARD);

		this.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		this.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		this.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		this.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		this.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		this.wrist.setDirection(Servo.Direction.FORWARD);
		this.rightClaw.setDirection(Servo.Direction.FORWARD);
		this.leftClaw.setDirection(Servo.Direction.REVERSE);

		this.wrist.setPosition(this.wristPosition);
		this.rightClaw.setPosition(this.leftClawPosition);
		this.leftClaw.setPosition(this.rightClawPosition);
	}
}
