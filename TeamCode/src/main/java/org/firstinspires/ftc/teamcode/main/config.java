package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.config.Config;

@Config
public class config {
	// - Movement config
	public final static double SPEED = 0.7d;
	public final static double ACCELERATION = 0.7d;
	public final static double DECELERATION = 0.3d;

	public final static double P_DRIVE_GAIN = 0.02d;
	public final static double I_DRIVE_GAIN = 0.001d;
	public final static double D_DRIVE_GAIN = 0.04d;

	public final static double HEADING_THRESHOLD = 1.5d;

	// - Mechanism config
	// - - Arm
	public final static double ARM_SPEED = 0.55d;
	public final static double ARM_BOOST = 0.8d;

	// - - Lift
	public final static double LIFT_SPEED = 0.75d;

	// - - Hand
	public final static double WRIST_GROUND = 0d;
	public final static double WRIST_MID = 0.1d;
	public final static double WRIST_SCORE = 0.45d;

	// - - Claw
	public final static double CLAW_OPEN = 0d;
	public final static double CLAW_CLOSE = 0.3d;

	// - - Rocket
	public final static double ROCKET_CLOSED = 0.45d;
	public final static double ROCKET_LAUNCHED = 0.0d;
}
