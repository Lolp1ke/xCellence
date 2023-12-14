package org.firstinspires.ftc.teamcode.autonomousOld;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class tag {
	private final LinearOpMode opMode;

	private VisionPortal visionPortal;
	private AprilTagProcessor _tag;

	public tag(final LinearOpMode _opMode) {
		this.opMode = _opMode;
	}


	public List<AprilTagDetection> detection() {
		List<AprilTagDetection> detections = _tag.getDetections();
		opMode.telemetry.addData("Tag detections: ", detections);

		return detections;
	}

	public void init() {
		_tag = new AprilTagProcessor.Builder()
			.setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
			.build();

		visionPortal = new VisionPortal
			.Builder()
			.setCameraResolution(new Size(1280, 720))
			.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
			.addProcessor(_tag)
			.enableLiveView(true)
			.build();
	}
}
