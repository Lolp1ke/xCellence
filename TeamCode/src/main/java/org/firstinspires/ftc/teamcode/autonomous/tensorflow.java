package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Deprecated
public class tensorflow {
	private final LinearOpMode opMode;

	private VisionPortal visionPortal;
	private TfodProcessor objectDetection;


	public tensorflow(LinearOpMode _opmode) {
		opMode = _opmode;
	}

	public List<Recognition> detection() {
		List<Recognition> objects = objectDetection.getRecognitions();
		opMode.telemetry.addData("Object detections: ", objects.size());

		for (Recognition object : objects) {
			opMode.telemetry.addData("Label: ", object.getLabel());
			opMode.telemetry.addData("Confidence: ", object.getConfidence());
		}

		return objects;
	}

	public void init() {
		objectDetection = new TfodProcessor
			.Builder()
			.build();

		visionPortal = new VisionPortal
			.Builder()
			.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
			.setCameraResolution(new Size(1280, 720))
			.addProcessor(objectDetection)
			.build();
	}
}
