package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Deprecated
public class movement4wd {
	private final LinearOpMode opMode;

	private DcMotor rightFront;
	private DcMotor leftFront;
	private DcMotor rightRear;
	private DcMotor leftRear;

	public movement4wd(final LinearOpMode _opMode) {
		opMode = _opMode;
	}


	public void run() {
		// Imagine coordinate plane
		double x = opMode.gamepad1.left_stick_x;
		double y = -opMode.gamepad1.left_stick_y;
		double turn = opMode.gamepad1.right_stick_x;

		// Tangent of the vector which goes through P(0, 0) to P(x, y) is angle to rotate a robot
		double angle = Math.atan2(y, x);
		double power = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)); // Power is just a hypothesis

		// y axe represents robots movement forward and backward, also initial heading of the robot is 0
		double sin = Math.sin(angle - Math.PI / 4);
		double cos = Math.cos(angle - Math.PI / 4);
		double max = Math.max(Math.abs(sin), Math.abs(cos));

		// Check how mecanum wheel work in youtube or ask your engineers
		double rightFrontPower = power * sin / max - turn;
		double leftFrontPower = power * cos / max + turn;
		double rightRearPower = power * cos / max - turn;
		double leftRearPower = power * sin / max + turn;

		// In case if power is exceeds value one
		if ((power + Math.abs(turn)) > 1) {
			rightFrontPower /= power + Math.abs(turn);
			leftFrontPower /= power + Math.abs(turn);
			rightRearPower /= power + Math.abs(turn);
			leftRearPower /= power + Math.abs(turn);
		}

		// Power up motors
		rightFront.setPower(rightFrontPower);
		leftFront.setPower(leftFrontPower);
		rightRear.setPower(rightRearPower);
		leftRear.setPower(leftRearPower);


		opMode.telemetry.addData("Right front", rightFrontPower);
		opMode.telemetry.addData("Left front", leftFrontPower);
		opMode.telemetry.addData("Right rear", rightRearPower);
		opMode.telemetry.addData("Left rear", leftRearPower);
	}

	// This one is also should work but I am not sure
	public void _run() {
		double normalizer;

		double x = opMode.gamepad1.left_stick_x;
		double y = -opMode.gamepad1.left_stick_y;
		double angle = opMode.gamepad1.right_stick_x;

		double rightFrontPower = -x + y - angle;
		double leftFrontPower = x + y + angle;
		double rightRearPower = x + y - angle;
		double leftRearPower = -x + y + angle;

		normalizer = Math.max(
			Math.max(Math.abs(rightFrontPower), Math.abs(leftFrontPower)),
			Math.max(Math.abs(rightRearPower), Math.abs(leftRearPower))
		);

		if (normalizer > 1) {
			rightFrontPower /= normalizer;
			leftFrontPower /= normalizer;
			rightRearPower /= normalizer;
			leftRearPower /= normalizer;
		}

		rightFront.setPower(rightFrontPower);
		leftFront.setPower(leftFrontPower);
		rightRear.setPower(rightRearPower);
		leftRear.setPower(leftRearPower);

		opMode.telemetry.addData("Front right/left: ", "%.2/%.2", rightFrontPower, leftFrontPower);
		opMode.telemetry.addData("Rear right/left: ", "%.2/%.2", rightRearPower, leftRearPower);
		opMode.telemetry.update();
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
	}
}
