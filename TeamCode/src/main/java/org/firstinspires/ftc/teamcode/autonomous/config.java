package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;

@Config
public class config {
	public final double SPEED = 0.4d;
	public final double TURN = 0.2d;


	private final double COUNTS_PER_MOTOR_REV = 28.0d;
	private final double DRIVE_GEAR_REDUCTION = 8.0d;
	public final double WHEEL_DIAMETER_CM = 9.0d;
	public final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415d);

	public final double P_DRIVE_GAIN = 0.03d;
	public final double P_TURN_GAIN = 0.02d;
	public final double HEADING_THRESHOLD = 1.0d;

	public final int CAMERA_WIDTH = 1280;
	public final int CAMERA_HEIGHT = 720;
}
