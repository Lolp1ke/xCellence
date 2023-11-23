package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "main", group = "!xCellence")
public class main extends LinearOpMode {
	private final movement2wd _movement2wd = new movement2wd(this);
	private final mechanism _mechanism = new mechanism(this);

	private boolean isTank = false;

	@Override
	public void runOpMode() {
		_mechanism.init();
		_movement2wd.init();

		telemetry.addData("Status: ", "vroom vroom");
		telemetry.update();

		while (opModeInInit()) {
			if (gamepad1.dpad_up) {
				isTank = true;
			} else if (gamepad1.dpad_down) {
				isTank = false;
			}
		}

		waitForStart();
		if (!isTank) {
			while (opModeIsActive()) {
				_mechanism.run();
				_movement2wd.car();

				telemetry.update();
			}
		} else {
			while (opModeIsActive()) {
				_mechanism.run();
				_movement2wd.tank();

				telemetry.update();
			}
		}
	}
}
