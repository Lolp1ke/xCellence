package org.firstinspires.ftc.teamcode.autonomous.arm;

import com.acmerobotics.dashboard.config.Config;

@Config
public class config {
	public static double ARM_POWER = 0.5d;
	public static int TICKS_PER_MOTOR_REV_ARM = 4;
	public static double GEAR_REDUCTION = 30d / 125d;
	public static double GEAR_RADIUS_CM = 4.328d;
	public static double COUNTS_PER_ANGLE = (TICKS_PER_MOTOR_REV_ARM * GEAR_REDUCTION / 3.46d) / (2d * GEAR_RADIUS_CM * Math.PI / 360d);
	public static double WRIST_GROUND = 0d;
	public static double WRIST_SCORE = 0.45d;
	public static double CLAW_OPEN = 0d;
	public static double CLAW_CLOSE = 0.3d;
}
