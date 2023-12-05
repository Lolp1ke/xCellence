package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class openCV {
	private final LinearOpMode opMode;
	private final config _config = new config();

	private OpenCvWebcam cvWebcam;
	public OpenCvPipeline cvPipeline = new pipeline();

	public openCV(final LinearOpMode _opMode) {
		opMode = _opMode;
	}

	public void init() {
		cvWebcam = OpenCvCameraFactory
			.getInstance()
			.createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"));

		cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				opMode.telemetry.addLine("Camera started");
				opMode.telemetry.update();
			}

			@Override
			public void onError(final int errorCode) {
				opMode.telemetry.addLine("Your code is shit");
				opMode.telemetry.addData("Error code: ", errorCode);
				opMode.telemetry.update();
			}
		});

		cvWebcam.setPipeline(cvPipeline);
		cvWebcam.startStreaming(_config.CAMERA_WIDTH,
			_config.CAMERA_HEIGHT,
			OpenCvCameraRotation.UPRIGHT);
	}
}

class pipeline extends OpenCvPipeline {
	Mat red = new Mat();

	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, red, Imgproc.COLOR_RGB2HSV);
		return red;
	}
}