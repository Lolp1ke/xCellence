package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.movement4wd;

@TeleOp(name = "Movement test", group = "test")
public class movement extends LinearOpMode {
	private final movement4wd _movement4wd = new movement4wd(this);
	GamepadEx gamepad = new GamepadEx(gamepad1);

	@Override
	public void runOpMode() {
		_movement4wd.init(hardwareMap);
//		gamepad = new GamepadEx(gamepad1);

		waitForStart();
		while (opModeIsActive()) {
			if (gamepad.isDown(GamepadKeys.Button.A)) {
				telemetry.addLine("a");
			}

			telemetry.update();
		}
	}
}
