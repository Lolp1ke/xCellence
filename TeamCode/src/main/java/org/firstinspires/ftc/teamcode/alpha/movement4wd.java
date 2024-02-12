package org.firstinspires.ftc.teamcode.alpha;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.imuUtil;
import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.pid;

public class movement4wd {
	private final config config = new config();
	private final VoltageSensor BATTERY_VOLTAGE_SENSOR;

	private final motorUtil motorUtil;
	private final imuUtil imuUtil;
	private final pid pid;

	private double rightRearPower = 0d;
	private double leftRearPower = 0d;
	private double rightFrontPower = 0d;
	private double leftFrontPower = 0d;
	private double fix = 0d;

	private boolean holdHeading = true;
	private double targetHeading = 0d;
	private double lastTurn = 0d;

	private double speedMultiplier = this.config.SPEED;

	private enum DRIVE_MODE {
		FIELD,
		ROBOT
	}

	private DRIVE_MODE driveMode = DRIVE_MODE.ROBOT;

	public movement4wd(final HardwareMap HARDWARE_MAP) {
		this.motorUtil = new motorUtil(
			HARDWARE_MAP,
			"right_rear",
			"left_rear",
			"right_front",
			"left_front"
		);

//		this.motorUtil = new motorUtil(
//			HARDWARE_MAP.get(DcMotorEx.class, "right_rear"),
//			HARDWARE_MAP.get(DcMotorEx.class, "left_rear"),
//			HARDWARE_MAP.get(DcMotorEx.class, "right_front"),
//			HARDWARE_MAP.get(DcMotorEx.class, "left_front")
//		);

		this.imuUtil = new imuUtil(HARDWARE_MAP);

		this.BATTERY_VOLTAGE_SENSOR = HARDWARE_MAP.voltageSensor.iterator().next();

		this.pid = new pid(
			this.config.P_VELOCITY_GAIN,
			this.config.I_VELOCITY_GAIN,
			this.config.D_VELOCITY_GAIN,
			this.config.PID_MAX_SPEED
		);
	}

	public void run(final Gamepad GAMEPAD) {
		if (GAMEPAD.dpad_up && GAMEPAD.right_trigger > 0.3d)
			this.driveMode = DRIVE_MODE.ROBOT;
		else if (GAMEPAD.dpad_down && GAMEPAD.right_trigger > 0.3d)
			this.driveMode = DRIVE_MODE.FIELD;

		if (this.driveMode == DRIVE_MODE.ROBOT)
			this.robotCentric(GAMEPAD);
		else this.fieldCentric(GAMEPAD);
	}

	public void robotCentric(final Gamepad GAMEPAD) {
		this.speedMultiplier = GAMEPAD.left_bumper ?
			this.config.DECELERATION : this.config.SPEED;

		double x = GAMEPAD.left_stick_x;
		double y = -GAMEPAD.left_stick_y;
		double turn = GAMEPAD.right_stick_x;

		double angle = Math.atan2(y, x);
		double power = Math.hypot(x, y);

		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);
		double max = Math.max(Math.abs(sin), Math.abs(cos));

		this.rightRearPower = (power * cos / max - turn);
		this.leftRearPower = (power * sin / max + turn);
		this.rightFrontPower = (power * sin / max - turn);
		this.leftFrontPower = (power * cos / max + turn);

		this.rightRearPower += this.fix;
		this.leftRearPower -= this.fix;
		this.rightFrontPower += this.fix;
		this.leftFrontPower -= this.fix;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.speedMultiplier, this.speedMultiplier);

		this.motorUtil.setPower(
			this.rightRearPower,
			this.leftRearPower,
			this.rightFrontPower,
			this.leftFrontPower
		);
	}

	public void fieldCentric(final Gamepad GAMEPAD) {
		double x = GAMEPAD.left_stick_x;
		double y = -GAMEPAD.left_stick_y;
		double turn = GAMEPAD.right_stick_x;

		double heading = -this.imuUtil.getHeading(AngleUnit.RADIANS);

		if (GAMEPAD.a)
			this.imuUtil.reset();

		if (turn == 0 && this.lastTurn != turn) {
			this.holdHeading = true;
			this.targetHeading = -this.imuUtil.getHeading(AngleUnit.RADIANS);
			this.pid.reset();
		} else if (turn != 0) {
			this.holdHeading = false;
		}
		this.lastTurn = turn;

		if (this.holdHeading)
			this.fix = this.pid.headingController(Math.toDegrees(-this.targetHeading), this.imuUtil.getHeading());
		else
			this.fix = 0d;

		double rotX = x * Math.cos(heading) - y * Math.sin(heading);
		double rotY = x * Math.sin(heading) + y * Math.cos(heading);

		this.rightRearPower = rotY + rotX - turn;
		this.leftRearPower = rotY - rotX + turn;
		this.rightFrontPower = rotY - rotX - turn;
		this.leftFrontPower = rotY + rotX + turn;

		this.rightRearPower += this.fix;
		this.leftRearPower -= this.fix;
		this.rightFrontPower += this.fix;
		this.leftFrontPower -= this.fix;

		this.rightRearPower = Range.clip(this.rightRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftRearPower = Range.clip(this.leftRearPower, -this.speedMultiplier, this.speedMultiplier);
		this.rightFrontPower = Range.clip(this.rightFrontPower, -this.speedMultiplier, this.speedMultiplier);
		this.leftFrontPower = Range.clip(this.leftFrontPower, -this.speedMultiplier, this.speedMultiplier);

		this.motorUtil.setPower(
			this.rightRearPower,
			this.leftRearPower,
			this.rightFrontPower,
			this.leftFrontPower
		);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Movement");

		TELEMETRY.addLine("Power");
		TELEMETRY.addData("Right rear: ", this.rightRearPower);
		TELEMETRY.addData("Left rear: ", this.leftRearPower);
		TELEMETRY.addData("Right front: ", this.rightFrontPower);
		TELEMETRY.addData("Left front: ", this.leftFrontPower);
		TELEMETRY.addData("Fix: ", this.fix);
		TELEMETRY.addLine();

		TELEMETRY.addData("Multiplier: ", this.speedMultiplier);
		TELEMETRY.addLine();

		TELEMETRY.addData("Hold heading: ", this.holdHeading);
		TELEMETRY.addData("Target heading: ", this.targetHeading);
		TELEMETRY.addLine();

		TELEMETRY.addLine();
	}

	public void init(final HardwareMap HARDWARE_MAP) {
		this.motorUtil.setDirection(
			DcMotorSimple.Direction.FORWARD,
			DcMotorSimple.Direction.REVERSE,
			DcMotorSimple.Direction.FORWARD,
			DcMotorSimple.Direction.REVERSE
		);

		this.motorUtil.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);

		this.motorUtil.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		this.motorUtil.setVelocityPIDFCoefficients(
			this.config.P_VELOCITY_GAIN,
			this.config.I_VELOCITY_GAIN,
			this.config.D_VELOCITY_GAIN,
			this.config.F_VELOCITY_GAIN,
			this.BATTERY_VOLTAGE_SENSOR.getVoltage()
		);
	}
}
