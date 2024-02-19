package org.firstinspires.ftc.teamcode.omega.red;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.omega.mechanism;
import org.firstinspires.ftc.teamcode.omega.movement4wd;
import org.firstinspires.ftc.teamcode.omega.openCV.openCV;

@Disabled
//@Autonomous(name = "Red far", group = "!!!RED")
public class far extends LinearOpMode {
	private final openCV _openCV = new openCV(this, true);
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
		int location = _openCV._pipeline.location;

		if (location == 1)
			left();
		else if (location == 2)
			center();
		else if (location == 3)
			right();

//		left();
//		center();
//		right();


		while (opModeIsActive()) {
		}
	}

	private void right() {
		_movement4wd.forward(63, 0);
		_movement4wd.rotate(90);
		_movement4wd.forward(-45, 90);

		_mechanism.purple();
		_movement4wd.forward(-60, 90);
		_movement4wd.rotate(45);

		_movement4wd.forward(-84, 45);
		_movement4wd.rotate(-90);
		_movement4wd.forward(40, -90);

		_mechanism.openRightClaw();

//		_movement4wd.forward(-150, 90);
//		_movement4wd.strafe(-25, 90);
//
//		_mechanism.yellow();
//		_movement4wd.strafe(-40, 90);
//		_movement4wd.forward(-27, 90);
	}

	private void center() {
		_movement4wd.forward(115, 0);
		_movement4wd.forward(-15, 0);
		_mechanism.purple();

		_movement4wd.forward(-90, 0);
		_movement4wd.rotate(-90);
		_movement4wd.forward(210, -90);

		_mechanism.openRightClaw();

//		_movement4wd.forward(115, 0);?
//		_movement4wd.rotate(180);
//		_mechanism.purple();
//
//		_movement4wd.forward(-15, 180);
//		_movement4wd.rotate(90);
//
//		_movement4wd.forward(-180, 90);
//		_movement4wd.strafe(-70, 90);
//		_mechanism.yellow();
//
//		_movement4wd.strafe(50, 90);
//		_movement4wd.forward(-30, 90);
	}

	private void left() {
		_movement4wd.forward(63, 0);
		_movement4wd.rotate(90);
		_movement4wd.strafe(-63, 90);

		_movement4wd.forward(-210, 90);
		_movement4wd.rotate(-90);
		_mechanism.openRightClaw();

//		_movement4wd.forward(63, 0);
//		_movement4wd.rotate(90);
//
//		_movement4wd.forward(15, 90);
//		_movement4wd.forward(-15, 90);
//		_mechanism.purple();
//
//		_movement4wd.forward(-200, 90);
//		_movement4wd.strafe(33, 90);
//		_mechanism.yellow();
//
//		_movement4wd.strafe(40, 90);
//		_movement4wd.forward(-25, 90);
	}
}
