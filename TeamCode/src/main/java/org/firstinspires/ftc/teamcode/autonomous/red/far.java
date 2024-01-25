package org.firstinspires.ftc.teamcode.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.mechanism;
import org.firstinspires.ftc.teamcode.autonomous.movement4wd;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Red far", group = "!!!RED")
public class far extends LinearOpMode {
	private final openCV _openCV = new openCV(this, true);
	private final movement4wd _movement4wd = new movement4wd(this);
	private final mechanism _mechanism = new mechanism(this);

	@Override
	public void runOpMode() {
		_openCV.init();

		_movement4wd.init(hardwareMap);
		_mechanism.init(hardwareMap);

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
		_movement4wd.forward(80, 0);
		_movement4wd.rotate(90);
		_movement4wd.forward(-50, 90);

		_mechanism.purple();
		_movement4wd.forward(-150, 90);
		_movement4wd.strafe(-20, 90);

		_mechanism.yellow();
		_movement4wd.strafe(50, 90);
		_movement4wd.forward(-30, 90);
	}

	private void center() {
		_movement4wd.forward(120, 0);
		_movement4wd.rotate(180);
		_mechanism.purple();

		_movement4wd.strafe(-180, 180);
		_movement4wd.rotate(90);
		_movement4wd.strafe(-50, 90);

		_mechanism.yellow();
		_movement4wd.strafe(50, 90);
		_movement4wd.forward(-30, 90);
	}

	private void left() {
		_movement4wd.forward(70, 0);
		_movement4wd.rotate(90);

		_movement4wd.forward(15, 90);
		_movement4wd.forward(-10, 90);
		_mechanism.purple();

		_movement4wd.forward(-200, 90);
		_movement4wd.strafe(33, 90);
		_mechanism.yellow();

		_movement4wd.strafe(40, 90);
		_movement4wd.forward(-25, 90);
	}
}
