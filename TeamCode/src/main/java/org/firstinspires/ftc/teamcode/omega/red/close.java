package org.firstinspires.ftc.teamcode.omega.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.omega.mechanism;
import org.firstinspires.ftc.teamcode.omega.movement4wd;
import org.firstinspires.ftc.teamcode.omega.openCV.openCV;

@Autonomous(name = "Red Close", group = "!!!RED")
public class close extends LinearOpMode {
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
		_movement4wd.resetHeading();

//		sleep(13000);

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
		_movement4wd.forward(65, 0);
		_movement4wd.rotate(90);
		_movement4wd.forward(-47, 90);

		_mechanism.purple();
		_movement4wd.strafe(-30, 90);
		_movement4wd.forward(-35, 90);

		_mechanism.yellow();
		_movement4wd.strafe(80, 90);
//		_movement4wd.forward(-7, 90);

		_mechanism.arm(-100);
	}

	private void center() {
		_movement4wd.forward(73, 0);
		_movement4wd.forward(-10, 0);
		_mechanism.purple();

		_movement4wd.forward(-10, 0);
		_movement4wd.rotate(90);
		_movement4wd.forward(-84, 90);
		_movement4wd.strafe(10, 90);

		_mechanism.yellow();
		_movement4wd.strafe(72, 90);
		_movement4wd.forward(-20, 90);

		_mechanism.arm(-100);
	}

	private void left() {
		_movement4wd.forward(65, 0);
		_movement4wd.rotate(90);

		_movement4wd.forward(15, 90);
		_movement4wd.forward(-14, 90);
		_mechanism.purple();

		_movement4wd.strafe(10, 90);
		_movement4wd.forward(-85, 90);
		_mechanism.yellow();

		_movement4wd.strafe(55, 90);
		_movement4wd.forward(-7, 90);

		_mechanism.arm(-100);
	}
}
