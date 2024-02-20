package org.firstinspires.ftc.teamcode.beta;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beta.hand.hand;
import org.firstinspires.ftc.teamcode.beta.movement.movement;

//@Disabled
@TeleOp(name = "detiler", group = "test")
public class main extends LinearOpMode {
	movement movement;
	hand hand;

	@Override
	public void runOpMode() {
		movement = new movement(hardwareMap);
		hand = new hand(hardwareMap);

		waitForStart();
		while (opModeIsActive()) {
			movement.run(gamepad1);
			hand.run(gamepad1);
		}
	}
}
