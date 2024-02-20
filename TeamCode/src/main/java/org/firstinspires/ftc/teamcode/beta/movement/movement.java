package org.firstinspires.ftc.teamcode.beta.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class movement {
	DcMotor leftMotor, rightMotor;

	double leftPower = 0;
	double rightPower = 0;
	private double range;


	public movement(HardwareMap hardwareMap) {
		leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
		rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

		leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
		rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
	}


	public void run(Gamepad gamepad) {
		double x = gamepad.right_stick_x;
		double y = -gamepad.left_stick_y;
		double leftPower = y - x;
		double rightPower = y + x;

		leftPower = Range.clip(leftPower, -1, 1);
		rightPower = Range.clip(rightPower, -1, 1);

		leftMotor.setPower(leftPower);
		rightMotor.setPower(rightPower);


	}
}

