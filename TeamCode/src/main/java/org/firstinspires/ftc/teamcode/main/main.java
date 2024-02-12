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

	private boolean isFieldCentric = false;


	@Override
	public void runOpMode() {
		_movement4wd.init(hardwareMap);
		_mechanism.init(hardwareMap);

		telemetry.addData("Status: ", "vroom vroom");
		telemetry.update();

		while (opModeInInit()) {
			if (gamepad1.dpad_down)
				isFieldCentric = true;
			else if (gamepad1.dpad_up)
				isFieldCentric = false;

			telemetry.addData("Drive mode: ", isFieldCentric ? "Field centric" : "Robot centric");
			telemetry.update();
		}

		waitForStart();
		while (opModeIsActive()) {
			if (isFieldCentric)
				_movement4wd.fieldCentric(gamepad1);
			else
				_movement4wd.robotCentric(gamepad1);

			_mechanism.run(gamepad2);

			_movement4wd.telemetry(telemetry);
			_mechanism.telemetry(telemetry);
			telemetry.update();
		}
	}
}
