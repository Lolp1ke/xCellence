package org.firstinspires.ftc.teamcode.sigma.hand;

import com.acmerobotics.dashboard.config.Config;

@Config("Hand config")
public class config {
	public static double WRIST_GROUND = 0d;
	public static double WRIST_MID = 0.255d;
	public static double WRIST_SCORE = 0.45d;

	public static double CLAW_OPEN = 0d;
	public static double CLAW_CLOSE = 0.3d;

	public enum WRIST {
		SCORE,
		GROUND,
		MID
	}

	public enum CLAW {
		OPEN,
		CLOSE
	}
}
