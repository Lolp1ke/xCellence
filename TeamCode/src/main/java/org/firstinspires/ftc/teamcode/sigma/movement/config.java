package org.firstinspires.ftc.teamcode.sigma.movement;

import com.acmerobotics.dashboard.config.Config;

@Config("Movement config")
public class config {
	public static double SPEED = 0.7d;
	public static double DECELERATION = 0.3d;


	public static double P_VELOCITY_GAIN = 48d;
	public static double I_VELOCITY_GAIN = 0d;
	public static double D_VELOCITY_GAIN = 17d;
	public static double F_VELOCITY_GAIN = -9d;

	public static double P_HEADING_GAIN = 0.02d;
	public static double I_HEADING_GAIN = 0d;
	public static double D_HEADING_GAIN = 0.04d;

	public static double PID_MAX_SPEED = 0.5d;

	protected enum DRIVE_MODE {
		FIELD,
		ROBOT
	}
}
