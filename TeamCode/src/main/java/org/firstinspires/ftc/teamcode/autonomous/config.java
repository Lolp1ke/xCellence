package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class config {
	public final int CAMERA_WIDTH = 320;
	public final int CAMERA_HEIGHT = 240;

	public final Scalar RED_HSV_LOW1_BOUNDARY = new Scalar(0.0d, 100.0d, 20.0d);
	public final Scalar RED_HSV_LOW2_BOUNDARY = new Scalar(160.0d, 100.0d, 20.0d);
	public final Scalar RED_HSV_HIGH1_BOUNDARY = new Scalar(10.0d, 255.0d, 255.0d);
	public final Scalar RED_HSV_HIGH2_BOUNDARY = new Scalar(179.0d, 255.0d, 255.0d);

	public final Scalar BLUE_HSV_LOW_BOUNDARY = new Scalar(90.0d, 50.0d, 70.0d);
	public final Scalar BLUE_HSV_HIGH_BOUNDARY = new Scalar(128.0d, 255.0d, 255.0d);
}
