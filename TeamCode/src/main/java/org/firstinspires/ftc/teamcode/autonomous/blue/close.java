package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.config;
import org.firstinspires.ftc.teamcode.autonomous.mechanism;
import org.firstinspires.ftc.teamcode.autonomous.movement;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Blue Close", group = "!BLUE")
public class close extends LinearOpMode {
	private final config _config = new config();
	private final openCV _openCV = new openCV(this, false);
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
		_movement.straight(_config.SPEED, -120, 0);
		_movement.turnFix(_config.TURN, 90, 1.5);

		_movement.straight(_config.SPEED, -20, 90);
		_movement.straight(_config.SPEED, 20, 90);

		_mechanism.placePurple();
		sleep(1000);

		_movement.straight(_config.SPEED, -100, 90);
		_movement.turnFix(_config.TURN, 180, 1.5);
		_movement.straight(_config.SPEED, -30, 180);

		_movement.turnFix(_config.TURN, -90, 1.5);
		_movement.straight(_config.SPEED, -30, -90);

		_mechanism.placeYellow();
		sleep(1000);
		_mechanism.resetHand();

		_movement.straight(_config.SPEED, -30, -90);
	}

	private void center() {
		_movement.straight(_config.SPEED, -150, 0);
		_mechanism.placePurple();
		sleep(1000);

		_movement.straight(_config.SPEED, 60, 0);
		_movement.turnFix(_config.TURN, -90, 1.5);

		_movement.straight(_config.SPEED, -70, -90);
		_mechanism.placeYellow();
		sleep(1000);

		_movement.straight(_config.SPEED, -30, -90);
	}

	private void left() {
	}
}
