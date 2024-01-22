package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class mechanism {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private DcMotor rightArm;
	private DcMotor leftArm;

	private Servo wrist;
	private Servo rightClaw;
	private Servo leftClaw;

	public mechanism(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void arm(final int angle) {
		int target = -(int) (angle * this._config.COUNTS_PER_ANGLE);

		this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		this.setTargetPosition(target);
		this.setPower(this._config.ARM_POWER);

		while (this.opMode.opModeIsActive() && this.isBusy()) ;

		this.setPower(0);
	}

	public void purple() {
		this.wrist.setPosition(_config.WRIST_GROUND);
		this.opMode.sleep(500);

		this.leftClaw.setPosition(_config.CLAW_OPEN);
		this.opMode.sleep(500);

		this.wrist.setPosition(_config.WRIST_SCORE);
		this.opMode.sleep(500);
	}

	public void yellow() {
		this.arm(175);

		this.rightClaw.setPosition(this._config.CLAW_OPEN);

		this.arm(-175);
	}

	private boolean isBusy() {
		return this.rightArm.isBusy() || this.leftArm.isBusy();
	}

	private void setMode(final DcMotor.RunMode runMode) {
		this.rightArm.setMode(runMode);
		this.leftArm.setMode(runMode);
	}

	private void setTargetPosition(final int rTarget, final int lTarget) {
		this.rightArm.setTargetPosition(rTarget);
		this.leftArm.setTargetPosition(lTarget);
	}

	private void setTargetPosition(final int target) {
		this.rightArm.setTargetPosition(target);
		this.leftArm.setTargetPosition(target);
	}

	private void setPower(final double rPower, final double lPower) {
		this.rightArm.setPower(rPower);
		this.leftArm.setPower(lPower);
	}

	private void setPower(final double power) {
		this.rightArm.setPower(power);
		this.leftArm.setPower(power);
	}

	public void init(final HardwareMap hardwareMap) {
		this.rightArm = hardwareMap.get(DcMotor.class, "right_arm");
		this.leftArm = hardwareMap.get(DcMotor.class, "left_arm");

		this.wrist = hardwareMap.get(Servo.class, "wrist");

		this.rightClaw = hardwareMap.get(Servo.class, "right_claw");
		this.leftClaw = hardwareMap.get(Servo.class, "left_claw");

		this.rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
		this.leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

		this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		this.rightArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.leftArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		this.wrist.setDirection(Servo.Direction.FORWARD);
		this.rightClaw.setDirection(Servo.Direction.FORWARD);
		this.leftClaw.setDirection(Servo.Direction.REVERSE);

		this.wrist.setPosition(_config.WRIST_SCORE);
		this.rightClaw.setPosition(_config.CLAW_CLOSE);
		this.leftClaw.setPosition(_config.CLAW_CLOSE);
	}
}
