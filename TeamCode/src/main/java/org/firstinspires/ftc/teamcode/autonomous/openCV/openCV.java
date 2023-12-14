package org.firstinspires.ftc.teamcode.autonomous.openCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.config;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class openCV {
	private final LinearOpMode opMode;
	private final config _config = new config();
	private final pipeline _pipeline;
	
	private OpenCvWebcam cvWebcam;
	
	
	public openCV(final LinearOpMode _opMode, final boolean isRed) {
		opMode = _opMode;
		_pipeline = new pipeline(isRed);
	}
	
	public void telemetry() {
		opMode.telemetry.addData("Camera FPS: ", cvWebcam.getFps());
		opMode.telemetry.addData("Pipeline Max FPS: ", cvWebcam.getCurrentPipelineMaxFps());
		opMode.telemetry.addData("Pipeline Latency: ", cvWebcam.getPipelineTimeMs());
	}
	
	public void init() {
		cvWebcam = OpenCvCameraFactory.getInstance()
				.createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
		cvWebcam.setPipeline(_pipeline);
		cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				cvWebcam.startStreaming(_config.CAMERA_WIDTH, _config.CAMERA_HEIGHT);
			}
			
			@Override
			public void onError(final int errorCode) {
				opMode.telemetry.addData("Error code: ", errorCode);
				opMode.telemetry.update();
			}
		});
	}
}
