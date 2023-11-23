package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "autonomous", group = "!xCellence")
public class autonomous extends LinearOpMode {
	private final tag _tag = new tag(this);
	private final movement _movement = new movement(this);
	private final config _config = new config();

	private boolean napoleonFound = false;
	private boolean napoleonLaunched = false;

	private int tagId = -1;

	@Override
	public void runOpMode() {
		_tag.init();
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
			for (AprilTagDetection tag : tags) {
				if (tag.metadata == null) continue;
				tagId = tag.id;

				telemetry.addData("id: ", tag.id);
				telemetry.addData("x: ", String.valueOf(tag.rawPose.x));
				telemetry.addData("y: ", String.valueOf(tag.rawPose.y));
				telemetry.addData("z: ", String.valueOf(tag.rawPose.z));
			}
			telemetry.update();
		}

		telemetry.clearAll();
		telemetry.addData("tagId: ", tagId);
		telemetry.update();

		waitForStart();
		_movement.straight(_config.SPEED, 30, 0);
//		while (opModeIsActive()) {
//		}
	}
}
