package org.firstinspires.ftc.teamcode.gamma;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.gamma.arm.arm;
import org.firstinspires.ftc.teamcode.gamma.hand.hand;
import org.firstinspires.ftc.teamcode.gamma.movement.movement4wd;
import org.firstinspires.ftc.teamcode.gamma.rocket.rocket;

@TeleOp(name = "main (gamma)", group = "test")
public class main extends LinearOpMode {
	private movement4wd movement4wd;
	private arm arm;
	private hand hand;
	private rocket rocket;


	public main() {
		super();

		this.movement4wd = new movement4wd(this.hardwareMap);
		this.arm = new arm(this.hardwareMap);
		this.hand = new hand(this.hardwareMap);
		this.rocket = new rocket(this.hardwareMap);
	}

	@Override
	public void runOpMode() {
//		this.movement4wd = new movement4wd(this.hardwareMap);
//		this.arm = new arm(this.hardwareMap);
//		this.hand = new hand(this.hardwareMap);

		while (opModeInInit()) {
			this.telemetry.addData("Status: ", "VROOM VROOM");
			this.telemetry.update();
		}

		waitForStart();
		while (opModeIsActive()) {
			this.movement4wd.run(this.gamepad1);
			this.arm.run(this.gamepad2);
			this.hand.run(this.gamepad2);
			this.rocket.run(this.gamepad1);


			this.movement4wd.telemetry(this.telemetry);
			this.arm.telemetry(this.telemetry);
			this.hand.telemetry(this.telemetry);
			this.rocket.telemetry(this.telemetry);
			this.telemetry.update();
		}
	}
}
