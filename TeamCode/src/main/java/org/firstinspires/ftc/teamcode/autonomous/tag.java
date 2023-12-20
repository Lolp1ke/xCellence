package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class tag {
	private final LinearOpMode opMode;

	private AprilTagProcessor aprilTag;
	private VisionPortal visionPortal;

	private boolean targetFound = false;
	private AprilTagDetection detectedTag = null;

	public boolean isInited = false;

	private int tagID;
	private final double DISTANCE_TO_TAG = 2d;

	public double drive = 0d;
	public double turn = 0d;

	public tag(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void run() {
		targetFound = false;

		List<AprilTagDetection> currentDetections = aprilTag.getDetections();
		for (AprilTagDetection detection : currentDetections) {
			if (detection.metadata != null) {
				if ((detection.id == tagID)) {
					targetFound = true;
					detectedTag = detection;
					break;
				} else {
					opMode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
				}
			}
		}

		if (targetFound) {
			opMode.telemetry.addData("Found", "ID %d (%s)", detectedTag.id, detectedTag.metadata.name);
			opMode.telemetry.addData("Range", "%5.1f cm", detectedTag.ftcPose.range * 2.54);
			opMode.telemetry.addData("Bearing", "%3.0f degrees", detectedTag.ftcPose.bearing);

			double rangeError = (detectedTag.ftcPose.range * 2.54 - DISTANCE_TO_TAG);
			double headingError = detectedTag.ftcPose.bearing;

			drive = Range.clip(rangeError * 0.02, -0.5d, 0.5d);
			turn = Range.clip(headingError * 0.01, -0.25d, 0.25d);

			opMode.telemetry.addData("Drive: ", drive);
			opMode.telemetry.addData("Turn: ", turn);
		} else {
			drive = 0d;
			turn = 0d;
		}
	}

	public void init(final int targetTagID) {
		aprilTag = new AprilTagProcessor
			.Builder()
			.build();
		aprilTag.setDecimation(2);

		visionPortal = new VisionPortal
			.Builder()
			.setCamera(opMode.hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag)
			.build();

		tagID = targetTagID;
		isInited = true;
	}
}
