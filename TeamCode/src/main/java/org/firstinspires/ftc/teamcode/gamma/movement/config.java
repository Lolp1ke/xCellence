package org.firstinspires.ftc.teamcode.gamma.movement;

import com.acmerobotics.dashboard.config.Config;

@Config("Movement config (gamma)")
public class config {
	protected final static double SPEED = 0.7d;
	protected final static double DECELERATION = 0.4d;


	protected final static double P_VELOCITY_GAIN = 48d;
	protected final static double I_VELOCITY_GAIN = 0d;
	protected final static double D_VELOCITY_GAIN = 17d;
	protected final static double F_VELOCITY_GAIN = -9d;

	protected final static double P_HEADING_GAIN = 0.02d;
	protected final static double I_HEADING_GAIN = 0.001d;
	protected final static double D_HEADING_GAIN = 0.04d;

	protected final static double PID_MAX_SPEED = 0.5d;

	protected enum DRIVE_MODE {
		FIELD,
		ROBOT
	}
}
