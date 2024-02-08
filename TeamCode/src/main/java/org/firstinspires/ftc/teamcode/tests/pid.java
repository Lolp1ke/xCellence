package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.oldAutonomous.config;

@TeleOp(name = "PID Tuner", group = "test")
public class pid extends LinearOpMode {
	private final config _config = new config();

	private DcMotorEx rightRear;
	private DcMotorEx leftRear;
	private DcMotorEx rightFront;
	private DcMotorEx leftFront;

	private IMU imu;

	private double P_GAIN = 0d;
	private double I_GAIN = 0d;
	private double D_GAIN = 0d;

	private final double P_TUNER = 0.0005d;
	private final double I_TUNER = 0.00005d;
	private final double D_TUNER = 0.0005d;

	private final double DISTANCE = 100d;

	private double headingError = 0d;
	private double lastError = 0d;
	private double totalError = 0d;

	private double turn = 0d;

	private boolean pGainPressed = false;
	private boolean lastPGainPressed = false;
	private boolean iGainPressed = false;
	private boolean lastIGainPressed = false;
	private boolean dGainPressed = false;
	private boolean lastDGainPressed = false;

	private boolean forwardPressed = false;
	private boolean lastForwardPressed = false;

	private boolean backwardPressed = false;
	private boolean lastBackwardPressed = false;

	private boolean rightPressed = false;
	private boolean lastRightPressed = false;

	private boolean leftPressed = false;
	private boolean lastLeftPressed = false;


	@Override
	public void runOpMode() {
		rightRear = hardwareMap.get(DcMotorEx.class, "right_rear");
		leftRear = hardwareMap.get(DcMotorEx.class, "left_rear");
		rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
		leftFront = hardwareMap.get(DcMotorEx.class, "left_front");

		imu = hardwareMap.get(IMU.class, "imu");

		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

		rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
					RevHubOrientationOnRobot.UsbFacingDirection.UP
				)
			)
		);
		imu.resetYaw();

		waitForStart();

		resetHeading();
		while (opModeIsActive()) {
			if ((pGainPressed = gamepad1.x) && !lastPGainPressed && !gamepad1.left_bumper)
				P_GAIN += P_TUNER;
			else if ((pGainPressed = gamepad1.x) && !lastPGainPressed && gamepad1.left_bumper)
				P_GAIN -= P_TUNER;

			if ((iGainPressed = gamepad1.a) && !lastIGainPressed && !gamepad1.left_bumper)
				I_GAIN += I_TUNER;
			else if ((iGainPressed = gamepad1.a) && !lastIGainPressed && gamepad1.left_bumper)
				I_GAIN -= I_TUNER;

			if ((dGainPressed = gamepad1.b) && !lastDGainPressed && !gamepad1.left_bumper)
				D_GAIN += D_TUNER;
			else if ((dGainPressed = gamepad1.b) && !lastDGainPressed && gamepad1.left_bumper)
				D_GAIN -= D_TUNER;

			lastPGainPressed = pGainPressed;
			lastIGainPressed = iGainPressed;
			lastDGainPressed = dGainPressed;


			if ((forwardPressed = gamepad1.dpad_up) && !lastForwardPressed)
				forward(true);
			else if ((backwardPressed = gamepad1.dpad_down) && !lastBackwardPressed)
				forward(false);

			lastForwardPressed = forwardPressed;
			lastBackwardPressed = backwardPressed;


			if ((rightPressed = gamepad1.dpad_right) && !lastRightPressed)
				strafe(true);
			else if ((leftPressed = gamepad1.dpad_left) && !lastLeftPressed)
				strafe(false);

			lastRightPressed = rightPressed;
			lastLeftPressed = leftPressed;

			telemetry();
		}
	}

	public void forward(final boolean FORWARD) {
		int target = (int) ((DISTANCE * (FORWARD ? 1d : -1d)) * _config.COUNTS_PER_CM);

		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMode(DcMotor.RunMode.RUN_TO_POSITION);

		setTargetPosition(target);
		setPower(_config.SPEED);

		PIDReset();
		while (opModeIsActive() && isBusy()) {
			turn = PIDControl(0, _config.SPEED, P_GAIN, I_GAIN, D_GAIN);

			if (!FORWARD)
				turn *= -1.0d;

			setPower(_config.SPEED + turn, _config.SPEED - turn, _config.SPEED + turn, _config.SPEED - turn);
		}

		setPower(0);
	}

	public void strafe(final boolean RIGHT) {
		int target = (int) (DISTANCE * (RIGHT ? 1d : -1d) * _config.COUNTS_PER_CM);

		setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		setMode(DcMotor.RunMode.RUN_TO_POSITION);

		setTargetPosition(target, -target, -target, target);
		setPower(_config.STRAFE, -_config.STRAFE, -_config.STRAFE, _config.STRAFE);

		PIDReset();
		while (opModeIsActive() && isBusy()) {
			turn = PIDControl(0, _config.STRAFE, P_GAIN, I_GAIN, D_GAIN);

			if (!RIGHT)
				turn *= -1.0d;

			setPower(_config.STRAFE + turn, -_config.STRAFE - turn, -_config.STRAFE - turn, _config.STRAFE + turn);
		}

		setPower(0);
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

	private boolean isBusy() {
		return rightRear.isBusy() && leftRear.isBusy() && rightFront.isBusy() && leftFront.isBusy();
	}

	private void telemetry() {
		telemetry.addLine("Heading");
		telemetry.addData("Current: ", getHeading());
		telemetry.addData("Heading error: ", headingError);
		telemetry.addData("Last error: ", lastError);
		telemetry.addData("Total error: ", totalError);
		telemetry.addData("Turn: ", turn);

		telemetry.addLine("Coefficients");
		telemetry.addData("P gain: ", P_GAIN);
		telemetry.addData("I gain: ", I_GAIN);
		telemetry.addData("D gain: ", D_GAIN);

		telemetry.update();
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
}
