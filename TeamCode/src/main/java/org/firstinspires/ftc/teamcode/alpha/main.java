package org.firstinspires.ftc.teamcode.alpha;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "new main (STILL IN TEST)", group = "test")
public class main extends LinearOpMode {
	private movement4wd movement4wd;
	private mechanism mechanism;


	@Override
	public void runOpMode() {
		this.movement4wd = new movement4wd(hardwareMap);
		this.mechanism = new mechanism(hardwareMap);

		waitForStart();
		while (opModeIsActive()) {
			this.movement4wd.run(gamepad1);
			this.mechanism.run(gamepad2);


			this.movement4wd.telemetry(telemetry);
			this.mechanism.telemetry(telemetry);
			telemetry.update();
		}
	}
}
