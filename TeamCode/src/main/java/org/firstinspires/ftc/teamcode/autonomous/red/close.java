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

		center();

		while (opModeIsActive()) {

//			_movement4wd.telemetry(telemetry);
//			telemetry.update();
		}
	}

	private void right() {
	}

	private void center() {
		_movement4wd.forward(90, 0);
		_movement4wd.forward(-10, 0);
		_mechanism.purple();

//		_movement4wd.forward(100, 0);
//		_movement4wd.rotate(-90);
//		_movement4wd.strafe(100, -90);

//		_movement4wd.rotate(90);
//		_movement4wd.strafe(-100, 90);

		_movement4wd.strafe(45, 0);
		_movement4wd.rotate(-90);
		_movement4wd.strafe(-10, -90);
		_mechanism.yellow();

//		_movement4wd.rotate(-90);
		_movement4wd.strafe(50, 0);
		_movement4wd.forward(-20, 0);
	}

	private void left() {
	}
}
