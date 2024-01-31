package org.firstinspires.ftc.teamcode.autonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.mechanism;
import org.firstinspires.ftc.teamcode.autonomous.movement4wd;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Blue far", group = "!!!BLUE")
public class far extends LinearOpMode {
	private final openCV _openCV = new openCV(this, false);
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

//		right();
//		center();
//		left();


		while (opModeIsActive()) {
		}
	}

	private void right() {
		_movement4wd.forward(63, 0);
		_movement4wd.rotate(-90);
		_mechanism.purple();
		_movement4wd.strafe(67, -90);

		_movement4wd.forward(-210, -90);
		_movement4wd.rotate(90);
		_mechanism.openRightClaw();

//		_movement4wd.forward(65, 0);
//		_movement4wd.rotate(-90);
//
//		_movement4wd.forward(15, -90);
//		_movement4wd.forward(-15, -90);
//		_mechanism.purple();
//
//		_movement4wd.forward(-215, -90);
//		_movement4wd.strafe(-30, -90);
//		_mechanism.yellow();
//
//		_movement4wd.strafe(-40, -90);
//		_movement4wd.forward(-25, -90);
	}

	private void center() {
		_movement4wd.forward(75, 0);
		_movement4wd.forward(-15, 0);
		_mechanism.purple();

		_movement4wd.forward(-65, 0);
		_movement4wd.rotate(90);
		_movement4wd.forward(210, 90);

		_mechanism.openRightClaw();

//		_movement4wd.forward(120, 0);
//		_movement4wd.rotate(180);
//		_mechanism.purple();
//
//		_movement4wd.forward(-15, 180);
//		_movement4wd.rotate(-90);
//
//		_movement4wd.forward(-200, -90);
//		_movement4wd.strafe(75, -90);
//		_movement4wd.forward(-15, -90);
//		_mechanism.yellow();
//
//		_movement4wd.strafe(-60, -90);
//		_movement4wd.forward(-30, -90);
	}

	private void left() {
		_movement4wd.forward(63, 0);
		_movement4wd.rotate(-90);
		_movement4wd.forward(-45, -90);

		_mechanism.purple();
		_movement4wd.forward(-60, -90);
		_movement4wd.rotate(-45);

		_movement4wd.forward(-84, -45);
		_movement4wd.rotate(90);
		_movement4wd.forward(30, 90);

		_mechanism.openRightClaw();


//		_movement4wd.forward(63, 0);
//		_movement4wd.rotate(-90);
//		_movement4wd.forward(-45, -90);
//
//		_mechanism.purple();
//		_movement4wd.forward(-160, -90);
//		_movement4wd.strafe(25, -90);
//
//		_mechanism.yellow();
//		_movement4wd.strafe(-80, -90);
//		_movement4wd.forward(-25, -90);
	}
}
