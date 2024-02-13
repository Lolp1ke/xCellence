package org.firstinspires.ftc.teamcode.alpha;

import com.acmerobotics.dashboard.config.Config;

@Config
public class config {
	public final static double SPEED = 0.8d;
	public final static double ACCELERATION = 1d;
	public final static double DECELERATION = 0.3d;

	public final static double P_VELOCITY_GAIN = 48d;
	public final static double I_VELOCITY_GAIN = 0d;
	public final static double D_VELOCITY_GAIN = 17d;
	public final static double F_VELOCITY_GAIN = -9d;
	public final static double PID_MAX_SPEED = 0.5d;

	public final static double ARM_SPEED = 0.55d;
	public final static double ARM_BOOST = 0.8d;

	public final static double LIFT_SPEED = 0.75d;
	public final static double LIFT_BOOST = 0.8d;

	public final static double CLAW_OPEN = 0d;
	public final static double CLAW_CLOSE = 0d;

	public final static double WRIST_GROUND = 0d;
	public final static double WRIST_MID = 0d;
	public final static double WRIST_SCORE = 0d;
}
