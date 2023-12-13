package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.config;
import org.firstinspires.ftc.teamcode.autonomous.mechanism;
import org.firstinspires.ftc.teamcode.autonomous.movement;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Blue Close KcellTech", group = "!BLUE")
public class closeKcellTech extends LinearOpMode {
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
		}
	}

	private void right() {
		_movement.straight(_config.SPEED, -55, 0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 3);

		_movement.straight(_config.SPEED, 20, 90);
		_movement.straight(_config.SPEED, -20, 90);

		_mechanism.placePurple();
		sleep(1000);

//		_movement.turn(_config.TURN, 83);
//		_movement.turnFix(_config.TURN, 83, 2);
//		_movement.straight(_config.SPEED, -85, 83);
//
//		_mechanism.placeYellow(-21);
//		sleep(1000);
//		_mechanism.resetHand();
	}

	private void center() {
		_movement.straight(_config.SPEED, -100, 0);
		_mechanism.placePurple();
		sleep(1000);
		_mechanism.encoderDrive(_config.ARM_SPEED, -3, -3, 2.0d);
		_mechanism.arm(-0.1d);

		_movement.straight(_config.SPEED, 40, 0);
		_mechanism.arm(0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 3.0d);

//		_movement.straight(_config.SPEED, -80, 90);
//		_mechanism.placeYellow(-21);
//		sleep(1000);
//		_mechanism.resetHand();
	}

	private void left() {
		_movement.straight(_config.SPEED, -60, 0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 2);

		_movement.straight(_config.SPEED, -40, 90);
		sleep(500);
		_mechanism.placePurple();
		sleep(500);

//		_movement.turn(_config.TURN, 95);
//		_movement.turnFix(_config.TURN, 95, 2.0d);
//		_movement.straight(_config.SPEED, -30, 95);
//
//		_mechanism.placeYellow(-25);
//		_mechanism.resetHand();
	}
}
