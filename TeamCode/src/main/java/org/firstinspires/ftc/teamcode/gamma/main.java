package org.firstinspires.ftc.teamcode.gamma;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.alpha.movement4wd;
import org.firstinspires.ftc.teamcode.gamma.mechanism.mechanism;

@TeleOp(name = "main (gamma)", group = "test")
public class main extends LinearOpMode {
	private movement4wd movement4wd;
	private mechanism mechanism;


	@Override
	public void runOpMode() {
		this.movement4wd = new movement4wd(this.hardwareMap);
		this.mechanism = new mechanism(this.hardwareMap);

		while (opModeInInit()) {
			this.telemetry.addData("Status: ", "VROOM VROOM");
			this.telemetry.update();
		}

		waitForStart();
		while (opModeIsActive()) {
			this.movement4wd.run(this.gamepad1);
			this.mechanism.run(this.gamepad2);


			this.movement4wd.telemetry(this.telemetry);
			this.mechanism.telemetry(this.telemetry);
			this.telemetry.update();
		}
	}
}
