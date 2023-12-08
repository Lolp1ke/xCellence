package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.mechanism;
import org.firstinspires.ftc.teamcode.autonomous.movement;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Autonomous Blue", group = "!Blue")
public class autonomousBlue extends LinearOpMode {
	private final openCV _openCV = new openCV(this, false);
	private final movement _movement = new movement(this);
	private final mechanism _mechanism = new mechanism(this);

	@Override
	public void runOpMode() {
		_openCV.init();
		_movement.init();
		_mechanism.init();


		while (opModeInInit()) {
			telemetry.addLine(String.valueOf(_openCV._pipeline._location));
			telemetry.update();
		}


		waitForStart();
		_movement.resetYaw();
	}
}
