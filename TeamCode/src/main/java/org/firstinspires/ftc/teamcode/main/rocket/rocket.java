package org.firstinspires.ftc.teamcode.main.rocket;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.servoUtil;

public @Config("Rocket config")
class rocket extends config {
	private final servoUtil servoUtil;

	private double position = CLOSED;

	public rocket(final HardwareMap HARDWARE_MAP) {
		this.servoUtil = new servoUtil(
			HARDWARE_MAP.get(Servo.class, "rocket")
		);

		this.servoUtil.setDirection(Servo.Direction.FORWARD);

		this.servoUtil.setPosition(this.position);
	}

	public void run(final Gamepad GAMEPAD) {
		if (GAMEPAD.x && GAMEPAD.a)
			this.position = LAUNCHED;

		this.servoUtil.setPosition(this.position);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("Rocket");
		TELEMETRY.addData("Position: ", "%.3f", this.position);
		TELEMETRY.addLine();

		TELEMETRY.addLine();
	}
}
