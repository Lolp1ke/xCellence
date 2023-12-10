package org.firstinspires.ftc.teamcode.autonomous.openCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonomous.config;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class openCV {
	private final LinearOpMode opMode;
	public final pipeline _pipeline;
	private final config _config = new config();
	private OpenCvWebcam cvWebcam;

	public openCV(final LinearOpMode _opMode, final Boolean isRed) {
		opMode = _opMode;
		_pipeline = new pipeline(isRed);
	}

	public void init() {
		cvWebcam = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));
		cvWebcam.setPipeline(_pipeline);
		cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				cvWebcam.startStreaming(
					_config.CAMERA_WIDTH,
					_config.CAMERA_HEIGHT,
					OpenCvCameraRotation.UPRIGHT
				);
			}

			@Override
			public void onError(int errorCode) {
				opMode.telemetry.addData("Error: ", errorCode);
				opMode.telemetry.update();
			}
		});
	}

	public void cameraOff() {
		cvWebcam.stopStreaming();
	}
}