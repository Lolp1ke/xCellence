package org.firstinspires.ftc.teamcode.alpha;

import com.acmerobotics.dashboard.config.Config;

@Config
public class config {
	public double SPEED = 0.8d;
	public double ACCELERATION = 1d;
	public double DECELERATION = 0.3d;

	public double P_VELOCITY_GAIN = 48d;
	public double I_VELOCITY_GAIN = 0d;
	public double D_VELOCITY_GAIN = 17d;
	public double F_VELOCITY_GAIN = -9d;
	public double PID_MAX_SPEED = 0.5d;

	public double ARM_SPEED = 0.55d;
	public double ARM_BOOST = 0.8d;

	public double LIFT_SPEED = 0.75d;
	public double LIFT_BOOST = 0.8d;

	public double CLAW_OPEN = 0d;
	public double CLAW_CLOSE = 0d;

	public double WRIST_GROUND = 0d;
	public double WRIST_MID = 0d;
	public double WRIST_SCORE = 0d;
}
