package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "Red", group = "!xCellence")
public class autonomousRed extends LinearOpMode {
	private final tag _tag = new tag(this);
	private final movement _movement = new movement(this);
	private final mechanism _mechanism = new mechanism(this);
	private final config _config = new config();

	private boolean napoleonFound = false;
	private boolean napoleonLaunched = false;

	private int tagId = 0;
	private double tagX;
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

		while (opModeInInit()) {
			List<AprilTagDetection> tags = _tag.detection();
			if (tags.size() > 0) {
				AprilTagDetection detectedTag = tags.get(0);
				tagId = detectedTag.id;
				tagX = detectedTag.rawPose.x;
				tagZ = detectedTag.rawPose.z;

				telemetry.addData("id: ", detectedTag.id);
				telemetry.addData("x: ", String.valueOf(detectedTag.rawPose.x));
				telemetry.addData("y: ", String.valueOf(detectedTag.rawPose.y));
				telemetry.addData("z: ", String.valueOf(detectedTag.rawPose.z));
				telemetry.update();
				break;
			}

			_movement.sendTelemetry(true);
		}

		telemetry.addData("tagZ: ", tagZ);
		telemetry.addData("tagX: ", tagX);
		telemetry.update();

		waitForStart();
		if (tagX > 0) {
			right();
		} else if (tagX < 0) {
			middle();
		} else {
			left();
		}
	}

	private void right() {
		_movement.straight(_config.SPEED, -90, 0);
		_movement.turn(_config.TURN, 90);
		_movement.turnFix(_config.TURN, 90, 1.5);

		sleep(1000);
		_mechanism.placePurple();

		_movement.straight(_config.SPEED, 70, 90);
		_movement.turn(_config.TURN, 0);
		_movement.turnFix(_config.TURN, 0, 1.5);

		_movement.straight(_config.SPEED, 40, 0);
		_movement.turn(_config.TURN, -90);
		_movement.turnFix(_config.TURN, -90, 1.5);

		_movement.straight(_config.SPEED, -27, -90);

		sleep(1000);
		_mechanism.placeYellow();
		_mechanism.resetHand();

		_movement.straight(_config.SPEED, -15, -90);
	}

	private void middle() {
		_movement.straight(_config.SPEED, -145, 0);

		_mechanism.placePurple();
		sleep(1000);

		_movement.straight(_config.SPEED, 60, 0);
		_movement.turn(_config.TURN, -90);
		_movement.turnFix(_config.TURN, -90, 2.0);

		_movement.straight(_config.SPEED, -96.5, -90);
		_mechanism.placeYellow();
		_mechanism.resetHand();
		_movement.straight(_config.SPEED, -10, -90);
	}

	private void left() {
		_movement.straight(_config.SPEED, -88, 0);
		_movement.turn(_config.TURN, -90);
		_movement.turnFix(_config.TURN, -90, 2.0);

		_movement.straight(_config.SPEED, 20, -90);
		_movement.straight(_config.SPEED, -15, -90);
		sleep(500);
		_mechanism.placePurple();

		_movement.straight(_config.SPEED, -110, -90);
		_mechanism.placeYellow();
		_mechanism.resetHand();
		_movement.straight(_config.SPEED, -13, -90);
	}
}