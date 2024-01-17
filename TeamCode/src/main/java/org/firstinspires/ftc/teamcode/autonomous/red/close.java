package org.firstinspires.ftc.teamcode.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.movement2wd;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;
import org.firstinspires.ftc.teamcode.autonomous.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.tag;

@Autonomous(name = "Red Close", group = "!!!RED")
public class close extends LinearOpMode {
	private final openCV _openCV = new openCV(this, true);
	private final tag _tag = new tag(this);
	private final movement2wd _movement = new movement2wd(this);


	@Override
	public void runOpMode() {
		_movement.init();
		_openCV.init();

		_movement.resetHeading();

		SampleMecanumDrive robot = new SampleMecanumDrive(hardwareMap);


		while (opModeInInit()) {
			_openCV.telemetry();
			_openCV._pipeline.telemetry(telemetry);

			telemetry.update();
		}


		waitForStart();
		_openCV.cameraOff();
		_tag.init(_openCV._pipeline.location + 3);
		while (opModeIsActive()) {
			_tag.run();
			_movement.move(_tag.drive, _tag.turn);

			telemetry.update();
		}
	}
}
