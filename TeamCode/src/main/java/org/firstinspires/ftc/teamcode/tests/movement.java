package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.omega.movement4wd;

@TeleOp(name = "Movement test", group = "test")
public class movement extends LinearOpMode {
	private final movement4wd _movement4wd = new movement4wd(this);

	@Override
	public void runOpMode() {
		_movement4wd.init(hardwareMap);

		waitForStart();
		while (opModeIsActive()) {

			telemetry.update();
		}
	}
}
