package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class tag {
	private final LinearOpMode opMode;
	
	private AprilTagProcessor aprilTag;
	private VisionPortal visionPortal;
	
	public tag(final LinearOpMode _opMode) {
		opMode = _opMode;
	}
	
	public void init() {
		aprilTag = new AprilTagProcessor.Builder().build();
		aprilTag.setDecimation(3);
		
		visionPortal = new VisionPortal.Builder().setCamera(
				opMode.hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTag).build();
	}
}
