package org.firstinspires.ftc.teamcode.autonomous.openCV;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;


public class config {
	public static int CAMERA_WIDTH = 752;
	public static int CAMERA_HEIGHT = 416;
	public static Scalar RED_HSV_LOW1_BOUNDARY = new Scalar(0d, 100d, 20d);
	public static Scalar RED_HSV_LOW2_BOUNDARY = new Scalar(160d, 100d, 20d);
	public static Scalar RED_HSV_HIGH1_BOUNDARY = new Scalar(10d, 255d, 255d);
	public static Scalar RED_HSV_HIGH2_BOUNDARY = new Scalar(179d, 255d, 255d);
	public static Scalar BLUE_HSV_LOW_BOUNDARY = new Scalar(90d, 50d, 70d);
	public static Scalar BLUE_HSV_HIGH_BOUNDARY = new Scalar(128d, 255d, 255d);

	private static int RECT_SIZE = 50;
	public static Rect RIGHT_RECT_RED = new Rect(
		new Point((CAMERA_WIDTH * 5.5d) / 7d - RECT_SIZE, CAMERA_HEIGHT / 1.5d - RECT_SIZE),
		new Point((CAMERA_WIDTH * 5.5d) / 7d + RECT_SIZE, CAMERA_HEIGHT / 1.5d + RECT_SIZE)
	);

	public static Rect CENTER_RECT_RED = new Rect(
		new Point(CAMERA_WIDTH / 2.3d - RECT_SIZE, CAMERA_HEIGHT / 1.8d - RECT_SIZE),
		new Point(CAMERA_WIDTH / 2.3d + RECT_SIZE, CAMERA_HEIGHT / 1.8d + RECT_SIZE)
	);

	public static Rect LEFT_RECT_RED = new Rect(
		new Point((CAMERA_WIDTH / 14d) - RECT_SIZE, CAMERA_HEIGHT / 1.6d - RECT_SIZE),
		new Point((CAMERA_WIDTH / 14d) + RECT_SIZE, CAMERA_HEIGHT / 1.6d + RECT_SIZE)
	);
	public static Rect RIGHT_RECT_BLUE = new Rect(
		new Point(CAMERA_WIDTH - RECT_SIZE, CAMERA_HEIGHT / 1.5d - RECT_SIZE),
		new Point(CAMERA_WIDTH - RECT_SIZE, CAMERA_HEIGHT / 1.5d + RECT_SIZE)
	);
	public static Rect CENTER_RECT_BLUE = new Rect(
		new Point(CAMERA_WIDTH / 1.53d - RECT_SIZE, CAMERA_HEIGHT / 1.8d - RECT_SIZE),
		new Point(CAMERA_WIDTH / 1.53d + RECT_SIZE, CAMERA_HEIGHT / 1.8d + RECT_SIZE)
	);
	public static Rect LEFT_RECT_BLUE = new Rect(
		new Point((CAMERA_WIDTH / 3d) - RECT_SIZE, CAMERA_HEIGHT / 1.5d - RECT_SIZE),
		new Point((CAMERA_WIDTH / 3d) + RECT_SIZE, CAMERA_HEIGHT / 1.5d + RECT_SIZE)
	);

	public static double CONFIDENCE = 25d;
}
