package org.firstinspires.ftc.teamcode.autonomousOld.cringe;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomousOld.config;
import org.firstinspires.ftc.teamcode.autonomousOld.mechanism;
import org.firstinspires.ftc.teamcode.autonomousOld.movement;
import org.firstinspires.ftc.teamcode.autonomousOld.tag;

@Autonomous(name = "Blue Left", group = "xCellence")
@Disabled
@Deprecated
public class autonomousBlueLeft extends LinearOpMode {
	private final tag _tag = new tag(this);
	private final movement _movement = new movement(this);
	private final mechanism _mechanism = new mechanism(this);
	private final config _config = new config();

	private boolean napoleonFound = false;
	private boolean napoleonLaunched = false;

	private double tagX = 0;
	private double tagZ;

	@Override
	public void runOpMode() {
		_tag.init();
		_mechanism.init();
		_movement.init();

		int napoleonSoundID = hardwareMap.appContext.getResources().getIdentifier("napoleon", "raw", hardwareMap.appContext.getPackageName());

		if (napoleonSoundID != 0)
			napoleonFound = SoundPlayer.getInstance().preload(hardwareMap.appContext, napoleonSoundID);

		if (napoleonFound && !napoleonLaunched) {
			SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, napoleonSoundID);
			napoleonLaunched = true;
		}

//		while (opModeInInit()) {
//			List<AprilTagDetection> tags = _tag.detection();
//			if (tags.size() > 0) {
//				AprilTagDetection detectedTag = tags.get(0);
//				tagX = detectedTag.rawPose.x;
//				tagZ = detectedTag.rawPose.z;
//
//				telemetry.addData("id: ", detectedTag.id);
//				telemetry.addData("x: ", String.valueOf(detectedTag.rawPose.x));
//				telemetry.addData("y: ", String.valueOf(detectedTag.rawPose.y));
//				telemetry.addData("z: ", String.valueOf(detectedTag.rawPose.z));
//				telemetry.update();
//				break;
//			}

		_movement.sendTelemetry(true);
//		}

//		telemetry.addData("tagZ: ", tagZ);
//		telemetry.addData("tagX: ", tagX);
//		telemetry.update();
//
		waitForStart();
		left();
//		if (tagX == 0) {
//			left();
//		} else if (tagX < 3) {
//			middle();
//		} else if (tagX > 0) {
//			right();
//		}
	}

	private void right() {
		_movement.straight(_config.SPEED, -85, 0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 1.5);

		_movement.straight(_config.SPEED, 20, 90);
		_movement.straight(_config.SPEED, -15, 90);

		sleep(1000);
		_mechanism.placePurple();

		_movement.straight(_config.SPEED, -70, 90);
		_movement.turn(_config.TURN, 0);
		_movement.turnFix(_config.TURN, 0, 1.5);

		_movement.straight(_config.SPEED, -20, 0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 1.5);

		_movement.straight(_config.SPEED, -40, 90);

		sleep(1000);
		_mechanism.placeYellow();
		_mechanism.resetHand();
		_movement.straight(_config.SPEED, -15, 90);
	}

	private void middle() {
		_movement.straight(_config.SPEED, -145, 0);

		_mechanism.placePurple();
		sleep(1000);

		_movement.straight(_config.SPEED, 55, 0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 2.0);

		_movement.straight(_config.SPEED, -100, 90);
		_mechanism.placeYellow();
		_mechanism.resetHand();
		_movement.straight(_config.SPEED, -10, 90);
	}

	private void left() {
		_movement.straight(_config.SPEED, -90, 0);
		_movement.turn(_config.TURN, -90);
		_movement.turnFix(_config.TURN, -90, 2.0);

		_movement.straight(_config.SPEED, 20, -90);
		_movement.straight(_config.SPEED, -15, -90);
		sleep(500);
		_mechanism.placePurple();

		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 1.5);

		_movement.straight(_config.SPEED, -70, 90);
		_movement.turn(_config.TURN, 0);
		_movement.turnFix(_config.TURN, 0, 1.5);

		_movement.straight(_config.SPEED, 20, 0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 1.5);

//		_movement.straight(_config.SPEED, -30, 90);
//		_movement.turn(_config.TURN, 90);
//		_movement.turnFix(_config.TURN, 90, 1.5);

		_mechanism.placeYellow();
		_mechanism.resetHand();
		_movement.straight(_config.SPEED, -10, 90);
	}
}