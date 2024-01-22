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
	private final movement4wd _movement4wd = new movement4wd();
	private final mechanism _mechanism = new mechanism();


	@Override
	public void runOpMode() {
		_movement4wd.init(hardwareMap);
		_mechanism.init(hardwareMap);

		telemetry.addData("Status: ", "vroom vroom");
		telemetry.update();

		waitForStart();
		while (opModeIsActive()) {
			_movement4wd.run(gamepad1);
			_mechanism.run(gamepad2);

			_movement4wd.telemetry(telemetry);
			_mechanism.telemetry(telemetry);
			telemetry.update();
		}
	}
}
