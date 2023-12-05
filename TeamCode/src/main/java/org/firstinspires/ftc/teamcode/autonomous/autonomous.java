package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Mat;

@Autonomous(name = "Autonomous dev", group = "!!!")
public class autonomous extends LinearOpMode {

	private openCV _openCV = new openCV(this);
	private mechanism _mechanism = new mechanism(this);
	private movement _movement = new movement(this);

	@Override
	public void runOpMode() {
		_openCV.init();
		_mechanism.init();
		_movement.init();

		telemetry.addData("Status: ", "kk");
		telemetry.update();

		waitForStart();
		while (opModeIsActive()) {
			_openCV.cvPipeline.processFrame(new Mat());
		}
	}
}
