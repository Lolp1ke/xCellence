package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "MecanumAutonomous", group = "Autonomous")
public class auto extends LinearOpMode {

	private DcMotor frontLeft, frontRight, backLeft, backRight;
	private BNO055IMU imu;
	private static final double DRIVE_SPEED = 0.6;
	private static final double TURN_SPEED = 0.5;

	private static final double P_DRIVE_COEFF = 0.03; // Proportional term for PID control
	private static final double I_DRIVE_COEFF = 0.001; // Integral term for PID control
	private static final double D_DRIVE_COEFF = 0.001; // Derivative term for PID control

	private static final double COUNTS_PER_REV = 28d; // Motor encoder counts per revolution
	private static final double DRIVE_GEAR_REDUCTION = 12d; // Gear reduction on the motor
	private static final double WHEEL_DIAMETER_INCHES = 9.6d; // Diameter of the wheel
	private static final double COUNTS_PER_INCH = (COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

	private static final double HEADING_P_COEFF = 0.03d; // Proportional term for heading PID control
	private static final double HEADING_I_COEFF = 0.002d; // Integral term for heading PID control
	private static final double HEADING_D_COEFF = 0.002d; // Derivative term for heading PID control

	private double headingIntegral = 0;
	private double lastHeadingError = 0;

	@Override
	public void runOpMode() {

		frontLeft = hardwareMap.dcMotor.get("left_front");
		frontRight = hardwareMap.dcMotor.get("right_front");
		backLeft = hardwareMap.dcMotor.get("left_rear");
		backRight = hardwareMap.dcMotor.get("right_rear");

		// Set motor directions
		frontLeft.setDirection(DcMotor.Direction.REVERSE);
		frontRight.setDirection(DcMotor.Direction.FORWARD);
		backLeft.setDirection(DcMotor.Direction.REVERSE);
		backRight.setDirection(DcMotor.Direction.FORWARD);

		// Reset encoders and set mode
		resetEncoders();
		runWithEncoder();

		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();
		imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		imu.initialize(imuParameters);

		waitForStart();

		driveDistance(24, DRIVE_SPEED);
		turnDegrees(90, TURN_SPEED);

		while (opModeIsActive()) {
			telemetry.addLine(String.valueOf(getHeading()));
			telemetry.update();
		}
		stopRobot();
	}

	private void driveDistance(double inches, double speed, double heading) {
		int targetPosition = (int) (inches * COUNTS_PER_INCH);
		setTargetPosition(targetPosition, targetPosition, targetPosition, targetPosition);
		runToPosition();

		while (opModeIsActive() && isBusy()) {
			// Calculate heading error
			double currentHeading = getHeading(); // Replace with your own method to get the current heading
			double headingError = heading - currentHeading;

			// Implement PID control for heading
			double headingCorrection = (HEADING_P_COEFF * headingError)
				+ (HEADING_I_COEFF * headingIntegral)
				+ (HEADING_D_COEFF * (headingError - lastHeadingError));

			// Update integral and derivative terms
			headingIntegral += headingError;
			lastHeadingError = headingError;

			// Adjust motor powers based on PID correction
			setPower(speed + headingCorrection, speed - headingCorrection, speed + headingCorrection, speed - headingCorrection);
		}

		stopRobot();
		runWithEncoder(); // Set motors back to RUN_WITHOUT_ENCODER mode
	}

	private void driveDistance(double inches, double speed) {
		int targetPosition = (int) (inches * COUNTS_PER_INCH);
		setTargetPosition(targetPosition, targetPosition, targetPosition, targetPosition);
		runToPosition();
		setPower(speed, speed, speed, speed);
		while (opModeIsActive() && isBusy()) {
			// Implement PID control here if needed
			// For simplicity, we're not using PID in this example
		}
		stopRobot();
		runWithEncoder(); // Set motors back to RUN_WITHOUT_ENCODER mode
	}

	private void turnDegrees(double degrees, double speed) {
		double turnDistance = Math.toRadians(degrees) * 10.16; // Assuming the distance from center to wheel is 10.16 cm
		int targetPosition = (int) (turnDistance * COUNTS_PER_INCH);
		setTargetPosition(targetPosition, -targetPosition, targetPosition, -targetPosition);
		runToPosition();
		setPower(speed, speed, speed, speed);
		while (opModeIsActive() && isBusy()) {
			// Implement PID control here if needed
			// For simplicity, we're not using PID in this example
		}
		stopRobot();
		runWithEncoder(); // Set motors back to RUN_WITHOUT_ENCODER mode
	}

	private double getHeading() {
		Orientation angles = imu.getAngularOrientation();
		return angles.firstAngle;
	}

	private void setTargetPosition(int frontLeft, int frontRight, int backLeft, int backRight) {
		this.frontLeft.setTargetPosition(frontLeft);
		this.frontRight.setTargetPosition(frontRight);
		this.backLeft.setTargetPosition(backLeft);
		this.backRight.setTargetPosition(backRight);
	}

	private void runToPosition() {
		this.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		this.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		this.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		this.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	private void setPower(double frontLeft, double frontRight, double backLeft, double backRight) {
		this.frontLeft.setPower(frontLeft);
		this.frontRight.setPower(frontRight);
		this.backLeft.setPower(backLeft);
		this.backRight.setPower(backRight);
	}

	private void resetEncoders() {
		this.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		this.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	}

	private void runWithEncoder() {
		this.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
	}

	private boolean isBusy() {
		return frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy();
	}

	private void stopRobot() {
		setPower(0, 0, 0, 0);
	}
}
