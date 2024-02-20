package org.firstinspires.ftc.teamcode.beta.hand;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class hand {
	Servo leftClaw, rightClaw, wrist;
	double x = 0;
	double clawPosition = 0;
	boolean lastA = false;

	public hand(HardwareMap hardwareMap) {
		leftClaw = hardwareMap.get(Servo.class, "leftClaw");
		rightClaw = hardwareMap.get(Servo.class, "rightClaw");
		wrist = hardwareMap.get(Servo.class, "wrist");

		leftClaw.setDirection(Servo.Direction.REVERSE);
		rightClaw.setDirection(Servo.Direction.FORWARD);

	}

	public void run(Gamepad gamepad) {
		boolean a = gamepad.a;


		if (a && !lastA) {
//			x++;
			if (clawPosition == 0)
				clawPosition = 1;
			else clawPosition = 0;
		}
		lastA = a;
//		if (x % 2 == 0) {
//			leftClaw.setPosition(0);
//			rightClaw.setPosition(0);
//		} else {
//			leftClaw.setPosition(1);
//			rightClaw.setPosition(1);
//		}

		leftClaw.setPosition(clawPosition);
		rightClaw.setPosition(clawPosition);

	}
}


