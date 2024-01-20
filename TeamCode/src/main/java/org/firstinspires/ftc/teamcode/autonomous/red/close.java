package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movement4wd;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;
import org.firstinspires.ftc.teamcode.autonomous.tag;

@Autonomous(name = "Red Close", group = "!!!RED")
@Config
public class close extends LinearOpMode {
	private final openCV _openCV = new openCV(this, true);
	private final tag _tag = new tag(this);
	private final movement4wd _movement4wd = new movement4wd(this);


	@Override
	public void runOpMode() {
		_openCV.init();
		_movement4wd.init();

		while (opModeInInit()) {
			_openCV.telemetry();
			_openCV._pipeline.telemetry(telemetry);

			telemetry.update();
		}

		waitForStart();
		_openCV.cameraOff();
		_tag.init(_openCV._pipeline.location + 3);
		_movement4wd.resetHeading();

//		_movement4wd.forward(100, 0);
//		_movement4wd.strafe(50, 0);
		_movement4wd.rotate(90);

		while (opModeIsActive()) {
//			_tag.run();
			_movement4wd.telemetry(telemetry);

			telemetry.update();
		}
	}
}
