package org.firstinspires.ftc.teamcode.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.config;
import org.firstinspires.ftc.teamcode.autonomous.mechanism;
import org.firstinspires.ftc.teamcode.autonomous.movement;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Red Far", group = "!RED")
public class far extends LinearOpMode {
	private final config _config = new config();
	private final openCV _openCV = new openCV(this, true);
	private final movement _movement = new movement(this);
	private final mechanism _mechanism = new mechanism(this);
	private int location;

	@Override
	public void runOpMode() {
		_openCV.init();
		_movement.init();
		_mechanism.init();


		while (opModeInInit()) {
			location = _openCV._pipeline._location;
			telemetry.addLine(String.valueOf(location));
			telemetry.update();
		}


		waitForStart();
		_openCV.cameraOff();
		_movement.resetYaw();

		switch (location) {
			case 1:
				right();
				break;
			case 2:
				center();
				break;
			case 3:
				left();
				break;
			default:
				if (Math.round(Math.random()) == 0) right();
				else left();
				break;
		}
	}

	private void right() {
	}

	private void center() {
	}

	private void left() {
	}
}
