package org.firstinspires.ftc.teamcode.aaaa;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "detiler", group = "test")
public class main extends LinearOpMode {
	DcMotor rightRear, leftRear, rightFront, leftFront;
	double rightRearPower = 0;
	double leftRearPower = 0;


	public void runOpMode() {
		rightRear = hardwareMap.get(DcMotor.class, "right_rear");
		leftRear = hardwareMap.get(DcMotor.class, "left_rear");
		rightFront = hardwareMap.get(DcMotor.class, "right_front");
		leftFront = hardwareMap.get(DcMotor.class, "left_front");

		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

		waitForStart();
		while (opModeIsActive()) {
			double y = -gamepad1.left_stick_y;
			double x = gamepad1.right_stick_x;

			rightRearPower = y - x;
			leftRearPower = y + x;

			rightRearPower = Range.clip(rightRearPower, -1, 1);
			leftRearPower = Range.clip(leftRearPower, -1, 1);

			rightRear.setPower(rightRearPower);
			leftRear.setPower(leftRearPower);

		}
	}
}
