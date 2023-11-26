package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Deprecated
public class tenserflow {
	private final LinearOpMode opmode;

	private VisionPortal visionPortal;
	private TfodProcessor objectDetection;

	private final String[] labels = {
		"Pixel"
	};

	public tenserflow(LinearOpMode _opmode) {
		opmode = _opmode;
	}

	public List<Recognition> detection() {
		List<Recognition> objects = objectDetection.getRecognitions();
		opmode.telemetry.addData("Object detections: ", objects.size());

		for (Recognition object : objects) {
			opmode.telemetry.addData("Label: ", object.getLabel());
			opmode.telemetry.addData("Confidence: ", object.getConfidence());
		}

		return objects;
	}

	public void init() {
		objectDetection = new TfodProcessor
			.Builder()
			.build();

		visionPortal = new VisionPortal
			.Builder()
			.setCamera(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"))
			.setCameraResolution(new Size(1280, 720))
			.addProcessor(objectDetection)
			.build();
	}
}
