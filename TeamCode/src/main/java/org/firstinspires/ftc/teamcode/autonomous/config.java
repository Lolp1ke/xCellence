package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

@Config
public class config {
	// Movement
	public final double SPEED = 0.4d;
	public final double STRAFE = 0.35d;
	public final double TURN = 0.25d;

	private final int TICKS_PER_MOTOR_REV = 28;
	private final double DRIVE_GEAR_REDUCTION = 10.5d;
	public final double WHEEL_RADIUS_CM = 4.8d;
	public final double COUNTS_PER_CM = (TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (2d * WHEEL_RADIUS_CM * Math.PI);

	public final double P_DRIVE_GAIN = 0.02d;
	public final double I_DRIVE_GAIN = 0.001d;
	public final double D_DRIVE_GAIN = 0.04d;


	public final double P_STRAFE_GAIN = 0.02d;
	public final double I_STRAFE_GAIN = 0.001d;
	public final double D_STRAFE_GAIN = 0.04d;

	public final double P_ROTATE_GAIN = 0.01d;
	public final double I_ROTATE_GAIN = 0.0005d;
	public final double D_ROTATE_GAIN = 0.02d;

	public final double HEADING_THRESHOLD = 1d;

	// Mechanism
	public final double ARM_POWER = 0.5d;
	private final int TICKS_PER_MOTOR_REV_ARM = 4;
	private final double GEAR_REDUCTION = 30d / 125d;
	public final double GEAR_RADIUS_CM = 4.328d;
	public final double COUNTS_PER_ANGLE = (TICKS_PER_MOTOR_REV_ARM * GEAR_REDUCTION / 3.46d) / (2d * GEAR_RADIUS_CM * Math.PI / 360d);
	public final double WRIST_GROUND = 0d;
	public final double WRIST_SCORE = 0.45d;
	public final double CLAW_OPEN = 0d;
	public final double CLAW_CLOSE = 0.3d;

	// Camera
	public final int CAMERA_WIDTH = 432;
	public final int CAMERA_HEIGHT = 240;
	public final Scalar RED_HSV_LOW1_BOUNDARY = new Scalar(0d, 100d, 20d);
	public final Scalar RED_HSV_LOW2_BOUNDARY = new Scalar(160d, 100d, 20d);
	public final Scalar RED_HSV_HIGH1_BOUNDARY = new Scalar(10d, 255d, 255d);
	public final Scalar RED_HSV_HIGH2_BOUNDARY = new Scalar(179d, 255d, 255d);
	public final Scalar BLUE_HSV_LOW_BOUNDARY = new Scalar(90d, 50d, 70d);
	public final Scalar BLUE_HSV_HIGH_BOUNDARY = new Scalar(128d, 255d, 255d);

	private final int RECT_SIZE = 30;
	public final Rect RIGHT_RECT = new Rect(
		new Point((CAMERA_WIDTH * 6d) / 7d - RECT_SIZE, CAMERA_HEIGHT / 2d - RECT_SIZE),
		new Point((CAMERA_WIDTH * 6d) / 7d + RECT_SIZE, CAMERA_HEIGHT / 2d + RECT_SIZE)
	);

	public final Rect CENTER_RECT = new Rect(
		new Point(CAMERA_WIDTH / 2d - RECT_SIZE, CAMERA_HEIGHT / 2d - RECT_SIZE),
		new Point(CAMERA_WIDTH / 2d + RECT_SIZE, CAMERA_HEIGHT / 2d + RECT_SIZE)
	);

	public final Rect LEFT_RECT = new Rect(
		new Point((CAMERA_WIDTH / 5d) - RECT_SIZE, CAMERA_HEIGHT / 2d - RECT_SIZE),
		new Point((CAMERA_WIDTH / 5d) + RECT_SIZE, CAMERA_HEIGHT / 2d + RECT_SIZE));
}