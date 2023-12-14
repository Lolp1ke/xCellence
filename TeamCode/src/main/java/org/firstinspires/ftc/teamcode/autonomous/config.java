package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class config {
	public final int CAMERA_WIDTH = 320;
	public final int CAMERA_HEIGHT = 240;
	public final Scalar RED_HSV_LOW1_BOUNDARY = new Scalar(0d, 100d, 20d);
	public final Scalar RED_HSV_LOW2_BOUNDARY = new Scalar(160d, 100d, 20d);
	public final Scalar RED_HSV_HIGH1_BOUNDARY = new Scalar(10d, 255d, 255d);
	public final Scalar RED_HSV_HIGH2_BOUNDARY = new Scalar(179d, 255d, 255d);
	public final Scalar BLUE_HSV_LOW_BOUNDARY = new Scalar(90d, 50d, 70d);
	public final Scalar BLUE_HSV_HIGH_BOUNDARY = new Scalar(128d, 255d, 255d);
	
	private final int RECT_SIZE = 30;
	public final Rect CENTER_RECT = new Rect(
			new Point((CAMERA_WIDTH * 6d) / 7d - RECT_SIZE, CAMERA_HEIGHT / 2d - RECT_SIZE),
			new Point((CAMERA_WIDTH * 6d) / 7d + RECT_SIZE, CAMERA_HEIGHT / 2d + RECT_SIZE));
	
	public final Rect LEFT_RECT = new Rect(
			new Point((CAMERA_WIDTH / 5d) - RECT_SIZE, CAMERA_HEIGHT / 2d - RECT_SIZE),
			new Point((CAMERA_WIDTH / 5d) + RECT_SIZE, CAMERA_HEIGHT / 2d + RECT_SIZE));
}
