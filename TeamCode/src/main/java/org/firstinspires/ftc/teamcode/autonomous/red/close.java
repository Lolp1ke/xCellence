package org.firstinspires.ftc.teamcode.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.autonomous.movement;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCv;
import org.firstinspires.ftc.teamcode.autonomous.tag;

@Autonomous(name = "Red Close", group = "!!!RED")
public class close extends LinearOpMode {
	private final openCv _openCv = new openCv(this, true);
	private final tag _tag = new tag(this);
	private final movement _movement = new movement(this);
	
	private DcMotorEx test;
	
	
	@Override
	public void runOpMode() {
		_movement.init();
		_openCv.init();
		
		while (opModeInInit()) {
			_openCv.telemetry();
			telemetry.update();
		}
		
		waitForStart();
		_movement.resetHeading();
		
		_openCv.cameraOff();
		_tag.init(_openCv._pipeline.location + 3);
		while (opModeIsActive()) {
			_tag.run();
			_movement.move(_tag.drive, _tag.turn);
			
			telemetry.update();
		}
	}
}
