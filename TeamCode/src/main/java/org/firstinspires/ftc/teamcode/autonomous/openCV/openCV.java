package org.firstinspires.ftc.teamcode.autonomous.openCV;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config("OpenCV config")
public class openCV extends config {

	private OpenCvWebcam cvWebcam;
	public final pipeline pipeline;

	public openCV(final boolean isRed, final HardwareMap HARDWARE_MAP) {
		this.pipeline = new pipeline(isRed);

		this.cvWebcam = OpenCvCameraFactory
			.getInstance()
			.createWebcam(HARDWARE_MAP.get(WebcamName.class, "Webcam 1"));

		this.cvWebcam.setPipeline(this.pipeline);

		this.cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				openCV.this.cvWebcam.startStreaming(openCV.CAMERA_WIDTH, openCV.CAMERA_HEIGHT,
					OpenCvCameraRotation.UPSIDE_DOWN);
			}

			@Override
			public void onError(final int errorCode) {
			}
		});
	}


	public void init(final HardwareMap hardwareMap) {
		this.cvWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
		this.cvWebcam.setPipeline(this.pipeline);

		this.cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				openCV.this.cvWebcam.startStreaming(openCV.CAMERA_WIDTH, openCV.CAMERA_HEIGHT,
					OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(final int errorCode) {
			}
		});
	}

	public void cameraOff(final Telemetry telemetry) {
		try {
			this.cvWebcam.stopStreaming();
			this.cvWebcam.closeCameraDeviceAsync(() -> telemetry.addLine("Camera device is now empty"));
		} catch (Exception error) {
			telemetry.addLine("Camera is not connected");
		}
	}

	public void telemetry(final Telemetry telemetry) {
		telemetry.addData("Camera FPS: ", this.cvWebcam.getFps());
		telemetry.addData("Pipeline Max FPS: ", this.cvWebcam.getCurrentPipelineMaxFps());
		telemetry.addData("Pipeline Latency: ", this.cvWebcam.getPipelineTimeMs());
	}
}
