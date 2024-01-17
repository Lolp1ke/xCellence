package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class movement4wd {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private DcMotor rightFront;
	private DcMotor leftFront;
	private DcMotor rightRear;
	private DcMotor leftRear;

	private IMU imu;

	private int rightFrontTarget;
	private int leftFrontTarget;
	private int rightRearTarget;
	private int leftRearTarget;

	private double turnSpeed = 0d;

	private double headingError = 0d;
	private double targetHeading = 0d;
	private double oldError = 0d;
	private double errorSum = 0d;

	public movement4wd(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void straight(final double driveSpeed, final double distance, final double heading) {
		if (!opMode.opModeIsActive()) return;

		int target = (int) (distance * _config.COUNTS_PER_CM);
		rightFrontTarget = rightFront.getCurrentPosition() + target;
		leftFrontTarget = leftFront.getCurrentPosition() + target;
		rightRearTarget = rightRear.getCurrentPosition() + target;
		leftRearTarget = leftRear.getCurrentPosition() + target;

		rightFront.setTargetPosition(rightFrontTarget);
		leftFront.setTargetPosition(leftFrontTarget);
		rightRear.setTargetPosition(rightRearTarget);
		leftRear.setTargetPosition(leftRearTarget);

		rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		move(Math.abs(driveSpeed), 0, 0);
		while (opMode.opModeIsActive() && rightFront.isBusy() && leftFront.isBusy() && rightRear.isBusy() && leftRear.isBusy()) {
			turnSpeed = PIDControl(heading);

			if (distance < 0)
				turnSpeed *= -1.0d;

			move(driveSpeed, turnSpeed, 0);

//			sendTelemetry(true);
		}

		move(0, 0, 0);
		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		opMode.sleep(1000);
	}

	public void move(final double x, final double y, final double turn) {
		double angle = Math.atan2(y, x);
		double power = Math.hypot(x, y);

		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);
		double max = Math.max(Math.abs(sin), Math.abs(cos));

		double rightFrontPower = (power * sin / max - turn);
		double leftFrontPower = (power * cos / max + turn);
		double rightRearPower = (power * cos / max - turn);
		double leftRearPower = (power * sin / max + turn);

		rightFrontPower = Range.clip(rightFrontPower, -1d, 1d);
		leftFrontPower = Range.clip(leftFrontPower, -1d, 1d);
		rightRearPower = Range.clip(rightRearPower, -1d, 1d);
		leftRearPower = Range.clip(leftRearPower, -1d, 1d);

		rightFront.setPower(rightFrontPower);
		leftFront.setPower(leftFrontPower);
		rightRear.setPower(rightRearPower);
		leftRear.setPower(leftRearPower);
	}

	private double getHeading() {
		return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	public void resetHeading() {
		imu.resetYaw();
	}

	private double PIDControl(final double heading) {
		targetHeading = heading;
		headingError = targetHeading - getHeading();

		while (headingError > 180) headingError -= 360;
		while (headingError <= -180) headingError += 360;

		oldError = headingError;
		errorSum += headingError;
		return Range.clip(headingError * _config.P_GAIN + errorSum * _config.I_GAIN + (headingError - oldError) * _config.D_GAIN, -1, 1);
	}

	public void init() {
		rightFront = opMode.hardwareMap.get(DcMotor.class, "right_front");
		leftFront = opMode.hardwareMap.get(DcMotor.class, "left_front");
		rightRear = opMode.hardwareMap.get(DcMotor.class, "right_rear");
		leftRear = opMode.hardwareMap.get(DcMotor.class, "left_rear");

		rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

		rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
	}
}
