package org.firstinspires.ftc.teamcode.main.movement;

public class config {
	public static double SPEED = 0.65d;
	public static double DECELERATION = 0.4d;

	public static double MIN_INPUT_POWER = 0.3d;


	public static double P_VELOCITY_GAIN = 48d;
	public static double I_VELOCITY_GAIN = 0d;
	public static double D_VELOCITY_GAIN = 17d;
	public static double F_VELOCITY_GAIN = -9d;

	public static double P_HEADING_GAIN = 0.013d; // 0.04d
	public static double I_HEADING_GAIN = 0d; // 0.0001d
	public static double D_HEADING_GAIN = 0d; // 0.0035d

	public static double PID_MAX_SPEED = 0.5d;
	public static int THRESHOLD_ERROR = 2;

	protected enum DRIVE_MODE {
		FIELD,
		ROBOT
	}
}
