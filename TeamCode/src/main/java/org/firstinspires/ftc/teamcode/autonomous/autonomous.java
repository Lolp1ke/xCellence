package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Autonomous Red", group = "!RED")
public class autonomous extends LinearOpMode {
	private final openCV _openCV = new openCV(this, true);
	private final movement _movement = new movement(this);
	private final mechanism _mechanism = new mechanism(this);

	@Override
	public void runOpMode() {
		_openCV.init();
		_movement.init();
		_mechanism.init();

		waitForStart();
		_movement.resetYaw();
		while (opModeInInit()) {
		}
	}
}
