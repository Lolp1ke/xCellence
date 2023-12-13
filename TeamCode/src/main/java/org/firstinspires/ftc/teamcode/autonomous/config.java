package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class config {
	// - Movement config part
	public final double SPEED = 0.35d;
	public final double TURN = 0.15d;

	private final double COUNTS_PER_MOTOR_REV = 28.0d;
	private final double DRIVE_GEAR_REDUCTION = 12.0d;
	public final double WHEEL_DIAMETER_CM = 9.0d;
	public final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI);


	// - - PID Control
	public final double P_DRIVE_GAIN = 0.03d;
	public final double P_TURN_GAIN = 0.028d;
	public final double D_TURN_GAIN = 0.15d;
	public final double HEADING_THRESHOLD = 1.0d;

	// - Mechanism config part
	private final double ARM_COUNTS_PER_MOTOR_REV = 288.0d;
	private final double ARM_DRIVE_GEAR_REDUCTION = 1.0d;
	private final double ARM_WHEEL_DIAMETER_INCHES = 9.5d / 2.54d;
	public final double ARM_COUNTS_PER_INCH = (ARM_COUNTS_PER_MOTOR_REV * ARM_DRIVE_GEAR_REDUCTION) /
		(ARM_WHEEL_DIAMETER_INCHES * Math.PI);

	public final double ARM_SPEED = 0.6d;

	public final double HAND_GROUND = 0.0d;
	public final double HAND_SCORE = 0.55d;

	public final double CLAW_OPENED = 0.05d;
	public final double CLAW_CLOSED = 0.3d;


	// - Autonomous config part
	public final int CAMERA_WIDTH = 320;
	public final int CAMERA_HEIGHT = 240;

	// - - Red color in HSV color range
	public final Scalar RED_HSV_LOW1_BOUNDARY = new Scalar(0.0d, 100.0d, 20.0d);
	public final Scalar RED_HSV_LOW2_BOUNDARY = new Scalar(160.0d, 100.0d, 20.0d);
	public final Scalar RED_HSV_HIGH1_BOUNDARY = new Scalar(10.0d, 255.0d, 255.0d);
	public final Scalar RED_HSV_HIGH2_BOUNDARY = new Scalar(179.0d, 255.0d, 255.0d);

	// - - Blue color in HSV color range
	public final Scalar BLUE_HSV_LOW_BOUNDARY = new Scalar(90.0d, 50.0d, 70.0d);
	public final Scalar BLUE_HSV_HIGH_BOUNDARY = new Scalar(128.0d, 255.0d, 255.0d);

	// - - Rectangles position and size
	public final int RECT_SIZE = 30;

	public final int RIGHT_X_POS = CAMERA_WIDTH * 4 / 5;
	public final int RIGHT_Y_POS = CAMERA_HEIGHT / 2;

	public final int CENTER_X_POS = CAMERA_WIDTH * 6 / 7;
	public final int CENTER_Y_POS = CAMERA_HEIGHT / 2;

	public final int LEFT_X_PIS = CAMERA_WIDTH / 5;
	public final int LEFT_Y_POS = CAMERA_HEIGHT / 2;
}