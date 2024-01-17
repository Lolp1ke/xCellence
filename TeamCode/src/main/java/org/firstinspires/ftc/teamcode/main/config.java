package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.config.Config;

@Config
public class config {
	// - Movement config
	public final double SPEED = 0.4d;
	public final double ACCELERATION = 0.7d;
	public final double DECELERATION = 0.3d;

	// - Mechanism config
	// - - Arm
	public final double ARM_SPEED = 0.55d;
	public final double ARM_BOOST = 0.8d;

	// - - Lift
	public final double LIFT_SPEED = 0.75d;

	// - - Hand
	public final double HAND_GROUND = 0d;
	public final double HAND_MID = 0.25d;
	public final double HAND_SCORE = 0.45d;

	// - - Claw
	public final double CLAW_OPEN = 0d;
	public final double CLAW_CLOSE = 0.3d;

	// - - Rocket
	public final double ROCKET_CLOSED = 0.45d;
	public final double ROCKET_LAUNCHED = 0.0d;
}
