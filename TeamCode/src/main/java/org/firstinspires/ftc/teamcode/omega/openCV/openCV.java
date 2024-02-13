package org.firstinspires.ftc.teamcode.omega.openCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.omega.config;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class openCV {
	public final pipeline _pipeline;
	private final LinearOpMode opMode;
	private final config _config = new config();

	private OpenCvWebcam cvWebcam;

	public openCV(final LinearOpMode _opMode, final boolean isRed) {
		opMode = _opMode;
		_pipeline = new pipeline(isRed);
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addData("Camera FPS: ", cvWebcam.getFps());
		telemetry.addData("Pipeline Max FPS: ", cvWebcam.getCurrentPipelineMaxFps());
		telemetry.addData("Pipeline Latency: ", cvWebcam.getPipelineTimeMs());
	}

	public void cameraOff() {
		try {
			cvWebcam.stopStreaming();
			cvWebcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
				@Override
				public void onClose() {
					opMode.telemetry.addLine("Camera device is now empty");
				}
			});
		} catch (Exception error) {
			opMode.telemetry.addLine("Camera is not connected");
		}
	}

	public void init() {
		cvWebcam = OpenCvCameraFactory.getInstance()
			.createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
		cvWebcam.setPipeline(_pipeline);
		cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				cvWebcam.startStreaming(_config.CAMERA_WIDTH, _config.CAMERA_HEIGHT,
					OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(final int errorCode) {
				opMode.telemetry.addData("Camera init error: ", errorCode);
				opMode.telemetry.update();
			}
		});
	}
}