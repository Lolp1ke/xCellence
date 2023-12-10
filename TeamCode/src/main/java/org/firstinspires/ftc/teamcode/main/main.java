package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Main is entry file
 * This class inherits LinearOpMode class from FTC SDK.
 * Then modules like @mechanism receive LinearOpMode's attributes (Read LinearOpMode class).
 */
@TeleOp(name = "main", group = "!xCellence")
public class main extends LinearOpMode {
	private final movement2wd _movement2wd = new movement2wd(this);
	private final mechanism _mechanism = new mechanism(this);


	@Override
	public void runOpMode() {
		_movement2wd.init();
		_mechanism.init();

		telemetry.addData("Status: ", "vroom vroom");
		telemetry.update();

		waitForStart();
		while (opModeIsActive()) {
			_movement2wd.car();
			_mechanism.run();

			telemetry.update();
		}
	}
}
