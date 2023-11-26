package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

public class _autonomousRed extends LinearOpMode {
	private final tenserflow _tenserflow = new tenserflow(this);

	@Override
	public void runOpMode() {
		_tenserflow.init();

		while (opModeInInit()) {
			List<Recognition> objects = _tenserflow.detection();
			if (objects.size() > 0) {
				Recognition object = objects.get(0);

				telemetry.addData("Label", object.getLabel());
				telemetry.addData("Confidence", object.getConfidence());
				telemetry.update();
			}
		}

		waitForStart();
	}
}
