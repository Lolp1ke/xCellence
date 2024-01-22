package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
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

	private MecanumDrive drive;

	private IMU imu;

	private double turn;

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
			turn = PIDControl(heading);

//			if (distance < 0)
//				turn *= -1.0d;

			setPower(_config.SPEED + turn, _config.SPEED - turn, _config.SPEED + turn, _config.SPEED - turn);
		}

		setPower(0);
		setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}

	public void strafe(double distance, final int heading) {
		int target = (int) (distance * _config.COUNTS_PER_CM);

		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMode(DcMotor.RunMode.RUN_TO_POSITION);

		setTargetPosition(target, -target, -target, target);
		setPower(_config.STRAFE, -_config.STRAFE, -_config.STRAFE, _config.STRAFE);

		while (opMode.opModeIsActive() && isBusy()) {
			turn = PIDControl(heading, _config.STRAFE, _config.P_GAIN, _config.I_GAIN, _config.D_GAIN);

//			if (distance < 0)
//				turn *= -1.0d;

			setPower(_config.STRAFE + turn, -_config.STRAFE - turn, -_config.STRAFE - turn, _config.STRAFE + turn);
		}

		setPower(0);
		setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rotate(heading);
	}

	public void left(final double distance, final int heading) {
		if (!opMode.opModeIsActive()) return;
//		distance += distance > 0 ? 20d : -20d;
		int target = (int) (distance * _config.COUNTS_PER_CM);

		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMode(DcMotor.RunMode.RUN_TO_POSITION);

		setTargetPosition(-target, target, target, -target);
//		setPower(_config.SPEED, -_config.SPEED, -_config.SPEED, _config.SPEED);
//		setPower(_config.SPEED);

		while (opMode.opModeIsActive() && isBusy()) {
			double turn = PIDControl(heading);

//			if (distance < 0)
//				turn *= -1.0d;

			setPower(-_config.SPEED - turn, _config.SPEED + turn, _config.SPEED + turn, -_config.SPEED - turn);
		}

		setPower(0);
	}

	public void rotate(int target) {
		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
//		timer.reset();
//		target *= -1;
//		target -= target > 0 ? 6 : -6;
		double headingDifference = (target - getHeading());

		while (opMode.opModeIsActive()
//			&& timer.time() < 5d
			&& (Math.abs(headingDifference) > _config.HEADING_THRESHOLD)) {
			headingDifference = (target - getHeading());

			turn = -PIDControl(target, _config.TURN, 0.005d, 0.0d, 0.0d);

//			if (headingDifference < 0) turnPower *= -1d;

			telemetry(opMode.telemetry);
			opMode.telemetry.update();
			setPower(-turn, turn, -turn, turn);
		}

		setPower(0);
//		opMode.sleep(500);
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

	private double PIDControl(final double HEADING, final double MAX_SPEED, final double P,
		final double I, final double D) {
		headingError = HEADING - getHeading();

		double finalError = headingError * P + totalError * I + (headingError - lastError) * D;

		lastError = headingError;
		totalError += headingError;
		return Range.clip(finalError, -MAX_SPEED, MAX_SPEED);
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
		telemetry.addData("Heading: ", getHeading());
		telemetry.addData("HeadingError: ", headingError);
		telemetry.addData("Last error: ", lastError);
		telemetry.addData("Total error: ", totalError);
		telemetry.addData("Turn power: ", turn);

		telemetry.addLine("Velocity");
		telemetry.addData("Right rear: ", rightRear.getVelocity());
		telemetry.addData("Left rear: ", leftRear.getVelocity());
		telemetry.addData("Right front: ", rightFront.getVelocity());
		telemetry.addData("Left front: ", leftFront.getVelocity());


		telemetry.addLine("Current position");
		telemetry.addData("Right rear: ", rightRear.getCurrentPosition());
		telemetry.addData("Left rear: ", leftRear.getCurrentPosition());
		telemetry.addData("Right front: ", rightFront.getCurrentPosition());
		telemetry.addData("Left front: ", leftFront.getCurrentPosition());

		telemetry.addLine("Tolerance");
		telemetry.addData("Right rear: ", rightRear.getTargetPositionTolerance());
		telemetry.addData("Left rear: ", leftRear.getTargetPositionTolerance());
		telemetry.addData("Right front: ", rightFront.getTargetPositionTolerance());
		telemetry.addData("Left front: ", leftFront.getTargetPositionTolerance());
	}

	public void init(final HardwareMap hardwareMap) {
		rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");
		leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
		rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
		leftFront = hardwareMap.get(DcMotorEx.class, "left_front");

		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		imu = hardwareMap.get(IMU.class, "imu");
		imu.initialize(new IMU.Parameters(
			new RevHubOrientationOnRobot(
				RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
				RevHubOrientationOnRobot.UsbFacingDirection.UP
			)
		));
		imu.resetYaw();
	}
}