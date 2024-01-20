package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class movement4wd {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private DcMotorEx rightRear;
	private DcMotorEx leftRear;
	private DcMotorEx rightFront;
	private DcMotorEx leftFront;

	private IMU imu;

	private double headingError = 0d;
	private double lastError = 0d;
	private double totalError = 0d;

	public movement4wd(final LinearOpMode _opMode) {
		opMode = _opMode;
	}


	public void forward(final double distance, final double heading) {
		int target = (int) (distance * _config.COUNTS_PER_CM);

		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMode(DcMotor.RunMode.RUN_TO_POSITION);

		setTargetPosition(target);
		setPower(_config.SPEED);

		while (opMode.opModeIsActive() && isBusy()) {
			double turn = PIDControl(heading);
			setPower(_config.SPEED + turn, _config.SPEED - turn, _config.SPEED + turn, _config.SPEED - turn);
		}

		setPower(0);
		opMode.sleep(1000);
	}

	public void strafe(final double distance, final double heading) {
		int target = (int) (distance * _config.COUNTS_PER_CM);

		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMode(DcMotor.RunMode.RUN_TO_POSITION);

		setTargetPosition(target, -target, -target, target);
		setPower(-_config.SPEED, _config.SPEED, _config.SPEED, -_config.SPEED);

		while (opMode.opModeIsActive() && isBusy()) {
			double turn = PIDControl(heading);
			setPower(_config.SPEED + turn, -_config.SPEED - turn, -_config.SPEED + turn, _config.SPEED + turn);
		}

		setPower(0);
		opMode.sleep(1000);
	}

	public void rotate(final int target) {
		double headingDifference = AngleUnit.normalizeDegrees(target - getHeading());
		ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
		timer.reset();

		while (opMode.opModeIsActive() && (Math.abs(headingDifference) > _config.HEADING_THRESHOLD)) {
//			double headingDifference = AngleUnit.normalizeDegrees(target - getHeading());

			double turnPower = Range.clip(PIDControl(target, true), -_config.TURN, _config.TURN);
			opMode.telemetry.addData("Target: ", target);
			opMode.telemetry.addData("Difference: ", headingDifference);
			opMode.telemetry.addData("Heading error: ", headingError);
			opMode.telemetry.addData("Turn power: ", turnPower);
			opMode.telemetry.addData("timer: ", timer.time());
			opMode.telemetry.update();

			setPower(turnPower, -turnPower, turnPower, -turnPower);
		}

		setPower(0);
		opMode.sleep(1000);
	}

	private double PIDControl(final double heading) {
		headingError = heading - getHeading();

		while (headingError > 180d) headingError -= 360d;
		while (headingError <= -180d) headingError += 360d;

		double finalError = headingError * _config.P_GAIN + totalError * _config.I_GAIN + (headingError - lastError) * _config.D_GAIN;

		lastError = headingError;
		totalError += headingError;
		return Range.clip(finalError, -1d, 1d);
	}

	private double PIDControl(final double heading, final boolean _) {
		headingError = heading - getHeading();

		while (headingError > 180d) headingError -= 360d;
		while (headingError <= -180d) headingError += 360d;

		double finalError = headingError * 0.005d + totalError * 0.0005d + (headingError - lastError) * 0.01d;

		lastError = headingError;
		totalError += headingError;
		return Range.clip(finalError, -1d, 1d);
	}

	private double getHeading() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	public void resetHeading() {
		imu.resetYaw();
	}

	private boolean isBusy() {
		return rightRear.isBusy() || leftRear.isBusy() || rightFront.isBusy() || leftFront.isBusy();
	}

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
		telemetry.addData("Heading", getHeading());
		telemetry.addData("HeadingError: ", headingError);
		telemetry.addData("Last error:", lastError);
		telemetry.addData("Total error:", totalError);
	}

	public void init() {
		rightRear = opMode.hardwareMap.get(DcMotorEx.class, "right_rear");
		leftRear = opMode.hardwareMap.get(DcMotorEx.class, "left_rear");
		rightFront = opMode.hardwareMap.get(DcMotorEx.class, "right_front");
		leftFront = opMode.hardwareMap.get(DcMotorEx.class, "left_front");

		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		imu = opMode.hardwareMap.get(IMU.class, "imu");
		imu.initialize(new IMU.Parameters(
			new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
				RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
			)
		));
		imu.resetYaw();
	}
}