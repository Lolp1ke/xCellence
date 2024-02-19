package org.firstinspires.ftc.teamcode.sigma;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.sigma.arm.arm;
import org.firstinspires.ftc.teamcode.sigma.hand.hand;
import org.firstinspires.ftc.teamcode.sigma.movement.movement;
import org.firstinspires.ftc.teamcode.sigma.rocket.rocket;
import org.firstinspires.ftc.teamcode.sigma.touch.touch;

@TeleOp(name = "main (sigma)", group = "test")
public class main extends LinearOpMode {
	private movement movement;
	private arm arm;
	private hand hand;
	private rocket rocket;
	private touch touch;

	@Override
	public void runOpMode() {
		this.movement = new movement(this.hardwareMap);
		this.arm = new arm(this.hardwareMap);
		this.hand = new hand(this.hardwareMap);
		this.rocket = new rocket(this.hardwareMap);
		this.touch = new touch(this.hardwareMap);

		this.waitForStart();
		while (this.opModeIsActive()) {
			this.movement.run(this.gamepad1);
			this.arm.run(this.gamepad2);
			this.hand.run(this.gamepad2);
			this.rocket.run(this.gamepad1);
			this.touch.run();


			this.movement.telemetry(this.telemetry);
			this.arm.telemetry(this.telemetry);
			this.hand.telemetry(this.telemetry);
			this.rocket.telemetry(this.telemetry);
			this.touch.telemetry(this.telemetry);
			this.telemetry.update();
		}
	}
}
