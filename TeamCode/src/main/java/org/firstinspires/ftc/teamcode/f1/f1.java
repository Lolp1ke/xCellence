package org.firstinspires.ftc.teamcode.f1;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "F1 STILL IN TEST", group = "!!!F1")
@Disabled
public class f1 extends LinearOpMode {
	private final config _config = new config();

	private DcMotorEx rightDrive;
	private DcMotorEx leftDrive;

	private IMU imu;

	private double rightPower = 0d;
	private double leftPower = 0d;

	private double drive = 0d;
	private double turn = 0d;

	private double targetHeading = 0d;


	private double headingError = 0d;
	private double oldError = 0d;
	private double errorSum = 0d;

	@Override
	public void runOpMode() {
		this.startEngine();

		this.telemetry.addLine("MCL60 is ready to drive");
		this.telemetry.update();
		this.waitForStart();
		this.imu.resetYaw();
		while (opModeIsActive()) {
			boolean isBoosted = this.gamepad1.right_bumper;
			boolean isSlowed = this.gamepad1.left_bumper;

			this.drive = -this.gamepad1.left_stick_y;
			this.turn = this.gamepad1.right_stick_x;

			this.rightPower = Range.scale(this.drive - this.turn, 0d, 2d, -1d, 1d);
			this.leftPower = Range.scale(this.drive + this.turn, 0d, 2d, -1d, 1d);

			double speedMultiplier = isBoosted ? (Math.abs(
				this.turn) > 0.2d ? this._config.SPEED : this._config.ACCELERATION) : (isSlowed ? this._config.DECELERATION : this._config.SPEED);

			this.rightDrive.setPower(this.rightPower * speedMultiplier);
			this.leftDrive.setPower(this.leftPower * speedMultiplier);
		}

		this.logTelemetry();
		this.telemetry.update();
	}

	private double PIDControl(final double heading) {
		this.targetHeading = heading;
		this.headingError = this.targetHeading - this.getHeading();

		while (this.headingError > 180d) this.headingError -= 360d;
		while (this.headingError <= -180d) this.headingError += 360d;

		this.oldError = this.headingError;
		this.errorSum += this.headingError;
		return Range.clip(
			this.headingError * this._config.P_GAIN + this.errorSum * this._config.I_GAIN + (this.headingError - this.oldError) * this._config.D_GAIN,
			-1d, 1d);
	}

	private double getHeading() {
		return this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
	}

	private void logTelemetry() {
		this.telemetry.addLine("Engine power");
		this.telemetry.addData("Right: ", this.rightPower);
		this.telemetry.addData("Left: ", this.leftPower);
		this.telemetry.addData("Drive: ", this.drive);
		this.telemetry.addData("Turn: ", this.turn);

		this.telemetry.addLine("Errors");
		this.telemetry.addData("Current error: ", this.headingError);
		this.telemetry.addData("Error sum: ", this.errorSum);
		this.telemetry.addData("Previous error: ", this.oldError);

		this.telemetry.addLine();
		this.telemetry.addData("Heading: ", this.getHeading());
		this.telemetry.addData("Target heading: ", this.targetHeading);
	}

	private void startEngine() {
		this.rightDrive = this.hardwareMap.get(DcMotorEx.class, "rightDrive");
		this.leftDrive = this.hardwareMap.get(DcMotorEx.class, "leftDrive");

		this.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
		this.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

		this.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		this.imu = this.hardwareMap.get(IMU.class, "imu");

		this.imu.initialize(new IMU.Parameters(
			new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
				RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));

		this.imu.resetYaw();
	}
}