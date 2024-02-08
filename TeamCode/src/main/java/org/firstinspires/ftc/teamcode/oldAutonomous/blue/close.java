package org.firstinspires.ftc.teamcode.oldAutonomous.blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.oldAutonomous.mechanism;
import org.firstinspires.ftc.teamcode.oldAutonomous.movement4wd;
import org.firstinspires.ftc.teamcode.oldAutonomous.openCV.openCV;

@Autonomous(name = "Blue close", group = "!!!BLUE")
public class close extends LinearOpMode {
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

		while (opModeIsActive()) {
		}
	}

	private void right() {

		_movement4wd.forward(65, 0);
		_movement4wd.rotate(-90);

		_movement4wd.forward(15, -90);
		_movement4wd.forward(-14, -90);
		_mechanism.purple();

		_movement4wd.strafe(-10, -90);
		_movement4wd.forward(-85, -90);
		_mechanism.yellow();

		_movement4wd.strafe(-55, -90);
		_movement4wd.forward(-7, -90);

		_mechanism.arm(-100);

//		_movement4wd.forward(65, 0);
//		_movement4wd.rotate(-90);
//
//		_movement4wd.forward(15, -90);
//		_movement4wd.forward(-15, -90);
//		_mechanism.purple();
//
//		_movement4wd.strafe(-10, -90);
//		_movement4wd.forward(-70, -90);
//		_mechanism.yellow();
//
//		_movement4wd.strafe(-40, -90);
//		_movement4wd.forward(-25, -90);
	}

	private void center() {
		_movement4wd.forward(73, 0);
		_movement4wd.forward(-10, 0);
		_mechanism.purple();

		_movement4wd.forward(-10, 0);
		_movement4wd.rotate(-90);
		_movement4wd.forward(-84, -90);
		_movement4wd.strafe(-10, -90);

		_mechanism.yellow();
		_movement4wd.strafe(-72, -90);
		_movement4wd.forward(-20, -90);

		_mechanism.arm(-100);

//		_movement4wd.forward(75, 0);
//		_movement4wd.forward(-15, 0);
//		_mechanism.purple();
//
//		_movement4wd.forward(-10, 0);
//		_movement4wd.rotate(-90);
//		_movement4wd.forward(-75, -90);
//		_movement4wd.strafe(-10, -90);
//		_mechanism.yellow();
//
//		_movement4wd.strafe(-70, -90);
//		_movement4wd.forward(-20, -90);
	}

	private void left() {
		_movement4wd.forward(65, 0);
		_movement4wd.rotate(-90);
		_movement4wd.forward(-47, -90);

		_mechanism.purple();
		_movement4wd.strafe(30, -90);
		_movement4wd.forward(-35, -90);

		_mechanism.yellow();
		_movement4wd.strafe(-80, -90);
//		_movement4wd.forward(-7, 90);

		_mechanism.arm(-100);

//		_movement4wd.forward(80, 0);
//		_movement4wd.rotate(-90);
//		_movement4wd.forward(-50, -90);
//
//		_mechanism.purple();
//		_movement4wd.strafe(-20, -90);
//		_movement4wd.forward(-30, -90);
//
//		_mechanism.yellow();
//		_movement4wd.strafe(-50, -90);
//		_movement4wd.forward(-30, -90);
	}
}
