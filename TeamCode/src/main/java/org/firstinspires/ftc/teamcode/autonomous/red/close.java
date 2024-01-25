package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.mechanism;
import org.firstinspires.ftc.teamcode.autonomous.movement4wd;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Red Close", group = "!!!RED")
@Config
public class close extends LinearOpMode {
	private final openCV _openCV = new openCV(this, true);
	//	private final tag _tag = new tag(this);

	private final movement4wd _movement4wd = new movement4wd(this);
	private final mechanism _mechanism = new mechanism(this);


	@Override
	public void runOpMode() {
		_openCV.init();

		_movement4wd.init(hardwareMap);
		_mechanism.init(hardwareMap);

		while (opModeInInit()) {
			_openCV.telemetry(telemetry);
			_openCV._pipeline.telemetry(telemetry);
			telemetry.update();
		}

		waitForStart();
		_openCV.cameraOff();
//		_tag.init(_openCV._pipeline.location + 3);
		_movement4wd.resetHeading();

		int location = _openCV._pipeline.location;
//		if (location == 1)
//			left();
//		else if (location == 2)
//			center();
//		else if (location == 3)
//			right();

		left();


		while (opModeIsActive()) {
		}
	}

	private void right() {
		_movement4wd.forward(80, 0);
		_movement4wd.rotate(-90);
		_movement4wd.forward(-60, -90);

		_mechanism.purple();
		_movement4wd.strafe(-30, 90);
		_movement4wd.forward(-50, 90);

		_mechanism.yellow();
		_movement4wd.strafe(60, 90);
		_movement4wd.forward(-30, 90);
	}

	private void center() {
		_movement4wd.forward(85, 0);
		_movement4wd.forward(-15, 0);
		_mechanism.purple();

		_movement4wd.strafe(60, 0);
		_movement4wd.rotate(90);
		_mechanism.yellow();

		_movement4wd.strafe(30, 90);
		_movement4wd.forward(-30, 90);
	}

	private void left() {
		_movement4wd.forward(70, 0);
		_movement4wd.rotate(90);

		_movement4wd.forward(15, 90);
		_movement4wd.forward(-15, 90);
		_mechanism.purple();

		_movement4wd.strafe(10, 90);
		_movement4wd.forward(-70, 90);
		_mechanism.yellow();

		_movement4wd.strafe(30, 90);
		_movement4wd.forward(-25, 90);
	}
}
