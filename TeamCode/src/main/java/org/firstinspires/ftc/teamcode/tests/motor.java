package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.motorUtil;

import java.util.HashMap;

@Config("Motor test config")
@TeleOp(name = "Motor test", group = "test")
public class motor extends LinearOpMode {
	private motorUtil motorUtil;


	private boolean isDpadUpPressed = false;
	private boolean isDpadDownPressed = false;

	public void runOpMode() {
		this.motorUtil = new motorUtil(
			this.hardwareMap.get(DcMotorEx.class, "right_rear"),
			this.hardwareMap.get(DcMotorEx.class, "left_rear"),
			this.hardwareMap.get(DcMotorEx.class, "right_front"),
			this.hardwareMap.get(DcMotorEx.class, "left_front")
		);

		this.motorUtil.setDirection(
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE,
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE
		);

		this.motorUtil.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		this.motorUtil.setZeroPowerBehaviour(DcMotorEx.ZeroPowerBehavior.BRAKE);


		this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


		double speed = 0.3d;
		boolean dpadUp;
		boolean dpadDown = false;

		this.waitForStart();
		while (this.opModeIsActive()) {
			if ((dpadUp = gamepad1.dpad_up) && !isDpadUpPressed && speed < 1d) {
				speed += 0.1d;
			} else if ((dpadDown = gamepad1.dpad_down) && !isDpadDownPressed && speed > -1d) {
				speed -= 0.1d;
			}
			isDpadUpPressed = dpadUp;
			isDpadDownPressed = dpadDown;

			if (gamepad1.b) {
				this.motorUtil.motors.get(2).setPower(speed);
			} else {
				this.motorUtil.motors.get(2).setPower(0);
			}

			if (gamepad1.x) {
				this.motorUtil.motors.get(3).setPower(speed);
			} else {
				this.motorUtil.motors.get(3).setPower(0);
			}

			if (gamepad1.y) {
				this.motorUtil.motors.get(0).setPower(speed);
			} else {
				this.motorUtil.motors.get(0).setPower(0);
			}

			if (gamepad1.a) {
				this.motorUtil.motors.get(1).setPower(speed);
			} else {
				this.motorUtil.motors.get(1).setPower(0);
			}

			if (gamepad1.dpad_right)
				this.motorUtil.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
			else if (gamepad1.dpad_left)
				this.motorUtil.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


			final HashMap<Integer, Double> velocities = this.motorUtil.getVelocities();
			this.telemetry.addLine("Right rear (y)");
			this.telemetry.addLine("Left rear (a)");
			this.telemetry.addLine("Right front (b)");
			this.telemetry.addLine("Left front (x)");
			this.telemetry.addLine();

			this.telemetry.addData("Speed: ", speed);
			this.telemetry.addLine();

			this.telemetry.addLine("Velocities");
			this.telemetry.addData("Right rear: ", velocities.get(0));
			this.telemetry.addData("Left rear: ", velocities.get(1));
			this.telemetry.addData("Right front: ", velocities.get(2));
			this.telemetry.addData("Left front: ", velocities.get(3));
			this.telemetry.addLine();

			this.telemetry.update();
		}
	}
}
