package org.firstinspires.ftc.teamcode.inst;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class colorDetection {
	private OpenCvWebcam cvWebcam;
	private pipeline cvPipeline;

	public void init(HardwareMap hardwareMap, Telemetry telemetry) {
		cvWebcam = OpenCvCameraFactory.getInstance()
			.createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
		cvWebcam.setPipeline(cvPipeline);
		cvWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				cvWebcam.startStreaming(320, 240,
					OpenCvCameraRotation.UPRIGHT);
			}

			@Override
			public void onError(int errorCode) {
				telemetry.addData("Camera init error: ", errorCode);
				telemetry.update();
			}
		});
	}

	public void cameraOff(Telemetry telemetry) {
		cvWebcam.stopStreaming();
		cvWebcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
			@Override
			public void onClose() {
				telemetry.addLine("Camera device is now empty");
				telemetry.update();
			}
		});
	}

	public void getColoredArea(Telemetry telemetry) {
		telemetry.addData("Colored area confidence: ", cvPipeline.filledColor);
	}
}


class pipeline extends OpenCvPipeline {
	Rect rectangle = new Rect(
		new Point(0, 0),
		new Point(10, 10)
	);

	Scalar blue = new Scalar(0, 0, 255);

	double filledColor = 0;

	@Override
	public Mat processFrame(Mat input) {
		Mat output = new Mat();
		Imgproc.cvtColor(input, output, Imgproc.COLOR_RGB2HSV);

		Core.inRange(output, new Scalar(90d, 50d, 70d), new Scalar(128d, 255d, 255d), output);

		Imgproc.cvtColor(output, output, Imgproc.COLOR_GRAY2RGB);
		Imgproc.rectangle(output, rectangle, blue);

		filledColor = Core.mean(new Mat(output, rectangle)).val[0];

		return output;
	}
}