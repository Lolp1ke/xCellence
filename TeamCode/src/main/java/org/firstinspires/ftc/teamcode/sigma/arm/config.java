package org.firstinspires.ftc.teamcode.sigma.arm;

import com.acmerobotics.dashboard.config.Config;

@Config("Arm config")
public class config {
	public static double ARM_SPEED = 0.55d;
	public static double ARM_BOOST = 0.8d;


	public static double LIFT_SPEED = 0.75d;
	public static double LIFT_BOOST = 0.9d;


	public static double P_POSITION_GAIN = 0.01d;
	public static double I_POSITION_GAIN = 0.01d;
	public static double D_POSITION_GAIN = 0.01d;
	public static double MAX_GAIN_SPEED = 0.3d;
}
