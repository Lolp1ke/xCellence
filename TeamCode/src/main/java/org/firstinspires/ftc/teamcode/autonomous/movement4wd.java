package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.motorUtil;

import java.util.Map;

public class movement4wd {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private motorUtil motors;

	private DcMotorEx rightRear;
	private DcMotorEx leftRear;
	private DcMotorEx rightFront;
	private DcMotorEx leftFront;

	private IMU imu;

	private double headingError = 0d;
	private double lastError = 0d;
	private double totalError = 0d;

	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;
	private double turn = 0d;


	public movement4wd(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void forward(final double DISTANCE, final int HEADING) {
		int target = (int) (DISTANCE * _config.COUNTS_PER_CM);

		motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		motors.setTargetPosition(target);
		motors.setPower(_config.SPEED);

		PIDReset();
		while (opMode.opModeIsActive() && motors.isBusy()) {
			turn = PIDControl(HEADING, _config.SPEED, _config.P_DRIVE_GAIN, _config.I_DRIVE_GAIN, _config.D_DRIVE_GAIN);

			if (DISTANCE < 0)
				turn *= -1.0d;

			rightRearPower = _config.SPEED + turn;
			leftRearPower = _config.SPEED - turn;
			rightFrontPower = _config.SPEED + turn;
			leftFrontPower = _config.SPEED - turn;

			motors.setPower(rightRearPower, leftRearPower, rightFrontPower, leftFrontPower);
			telemetry(opMode.telemetry);
		}

		motors.setPower(0);
		opMode.sleep(300);
		rotate(HEADING);
	}

	public void strafe(final double DISTANCE, final int HEADING) {
		int target = (int) (DISTANCE * _config.COUNTS_PER_CM);

		motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		motors.setTargetPosition(target, -target, -target, target);
		motors.setPower(_config.STRAFE, -_config.STRAFE, -_config.STRAFE, _config.STRAFE);

		PIDReset();
		while (opMode.opModeIsActive() && motors.isBusy()) {
			turn = PIDControl(HEADING, _config.STRAFE,
				_config.P_STRAFE_GAIN, _config.I_STRAFE_GAIN, _config.D_STRAFE_GAIN);

			if (DISTANCE < 0)
				turn *= -1.0d;

			rightRearPower = _config.STRAFE + turn;
			leftRearPower = -_config.SPEED - turn;
			rightFrontPower = -_config.STRAFE - turn;
			leftFrontPower = _config.STRAFE + turn;

			motors.setPower(rightRearPower, leftRearPower, rightFrontPower, leftFrontPower);
			telemetry(opMode.telemetry);
		}

		motors.setPower(0);
		opMode.sleep(200);
		rotate(HEADING);
	}

	public void rotate(final int TARGET) {
		motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		double headingDifference = TARGET - getHeading();
		ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
		timer.reset();

		PIDReset();
		while (opMode.opModeIsActive()
			&& timer.time() < 4d
			&& (Math.abs(headingDifference) > _config.HEADING_THRESHOLD)) {
			headingDifference = TARGET - getHeading();

			turn = PIDControl(TARGET, _config.TURN, _config.P_ROTATE_GAIN, _config.I_ROTATE_GAIN, _config.D_ROTATE_GAIN);

			motors.setPower(turn, -turn, turn, -turn);
			telemetry(opMode.telemetry);
		}

		motors.setPower(0);
		opMode.sleep(200);
	}

	private double PIDControl(final double HEADING, final double MAX_SPEED, final double P,
		final double I, final double D) {
		headingError = HEADING - getHeading();

		while (headingError > 180d) headingError -= 360d;
		while (headingError <= -180d) headingError += 360d;

		double finalError = headingError * P + totalError * I + (headingError - lastError) * D;

		lastError = headingError;
		totalError += headingError;
		return Range.clip(finalError, -MAX_SPEED, MAX_SPEED);
	}

	private void PIDReset() {
		headingError = 0d;
		lastError = 0d;
		totalError = 0d;
	}

	private double getHeading() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	public void resetHeading() {
		imu.resetYaw();
	}

//	private boolean isBusy() {
//		return rightRear.isBusy() && leftRear.isBusy() && rightFront.isBusy() && leftFront.isBusy();
//	}

	private void setMode(final DcMotor.RunMode runMode) {
		rightRear.setMode(runMode);
		leftRear.setMode(runMode);
		rightFront.setMode(runMode);
		leftFront.setMode(runMode);
	}

	private void setTargetPosition(final int target) {
		rightRear.setTargetPosition(target);
		leftRear.setTargetPosition(target);
		rightFront.setTargetPosition(target);
		leftFront.setTargetPosition(target);
	}

	private void setTargetPosition(final int rRear, final int lRear, final int rFront,
		final int lFront) {
		rightRear.setTargetPosition(rRear);
		leftRear.setTargetPosition(lRear);
		rightFront.setTargetPosition(rFront);
		leftFront.setTargetPosition(lFront);
	}

	private void setPower(final double power) {
		rightRear.setPower(power);
		leftRear.setPower(power);
		rightFront.setPower(power);
		leftFront.setPower(power);
	}

	private void setPower(final double rRear, final double lRear, final double rFront,
		final double lFront) {
		rightRear.setPower(rRear);
		leftRear.setPower(lRear);
		rightFront.setPower(rFront);
		leftFront.setPower(lFront);
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addLine("Heading");
		telemetry.addData("Current: ", getHeading());
		telemetry.addData("Heading error: ", headingError);
		telemetry.addData("Last error: ", lastError);
		telemetry.addData("Total error: ", totalError);
		telemetry.addLine();

		telemetry.addLine("Power");
		telemetry.addData("Right rear: ", rightRearPower);
		telemetry.addData("Left rear: ", leftRearPower);
		telemetry.addData("Right front: ", rightFrontPower);
		telemetry.addData("Left front: ", leftFrontPower);
		telemetry.addData("Turn power: ", turn);
		telemetry.addLine();

		telemetry.addLine("Current position");
		for (Map.Entry<Integer, Integer> entry : motors.getCurrentPositions().entrySet()) {
			telemetry.addData(String.valueOf(entry.getKey()), entry.getValue());
		}
//		telemetry.addData("Right rear: ", rightRear.getCurrentPosition());
//		telemetry.addData("Left rear: ", leftRear.getCurrentPosition());
//		telemetry.addData("Right front: ", rightFront.getCurrentPosition());
//		telemetry.addData("Left front: ", leftFront.getCurrentPosition());

//		telemetry.addLine("Velocity");
//		telemetry.addData("Right rear: ", rightRear.getVelocity());
//		telemetry.addData("Left rear: ", leftRear.getVelocity());
//		telemetry.addData("Right front: ", rightFront.getVelocity());
//		telemetry.addData("Left front: ", leftFront.getVelocity());

//		telemetry.addLine("Tolerance");
//		telemetry.addData("Right rear: ", rightRear.getTargetPositionTolerance());
//		telemetry.addData("Left rear: ", leftRear.getTargetPositionTolerance());
//		telemetry.addData("Right front: ", rightFront.getTargetPositionTolerance());
//		telemetry.addData("Left front: ", leftFront.getTargetPositionTolerance());

		telemetry.update();
	}

	public void init(final HardwareMap hardwareMap) {
//		rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");
//		leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
//		rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
//		leftFront = hardwareMap.get(DcMotorEx.class, "left_front");

		motors = new motorUtil(
			rightRear, leftRear,
			rightFront, leftFront
		);

		motors.init(hardwareMap,
			"right_rear", "left_rear",
			"right_front", "left_front"
		);

		motors.setDirection(
			DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE,
			DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE
		);

//		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
//		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

		motors.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

//		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		motors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//		setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
					RevHubOrientationOnRobot.UsbFacingDirection.UP
				)
			));
		imu.resetYaw();
	}
}