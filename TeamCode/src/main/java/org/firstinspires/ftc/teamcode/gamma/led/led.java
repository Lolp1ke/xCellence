package org.firstinspires.ftc.teamcode.gamma.led;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class led {
	private ColorSensor rightClaw;
	private ColorSensor leftClaw;

	private List<Integer> leftRGBAValues;
	private List<Integer> rightRGBAValues;

	private PWMOutput wtfIsThis;

	public led(final HardwareMap HARDWARE_MAP) {
		this.rightClaw = HARDWARE_MAP.get(ColorSensor.class, "right");
		this.leftClaw = HARDWARE_MAP.get(ColorSensor.class, "left");

		this.leftClaw.enableLed(true);
		this.rightClaw.enableLed(true);

		this.wtfIsThis = HARDWARE_MAP.pwmOutput.get("wtf");
	}

	public void run() {
		this.rightRGBAValues = Arrays.asList(
			this.rightClaw.red(),
			this.rightClaw.green(),
			this.rightClaw.blue(),
			this.rightClaw.alpha()
		);

		this.leftRGBAValues = Arrays.asList(
			this.leftClaw.red(),
			this.leftClaw.green(),
			this.leftClaw.blue(),
			this.leftClaw.alpha()
		);

		wtfIsThis.setPulseWidthOutputTime(1);
	}

	public void telemetry(final Telemetry TELEMETRY) {
		TELEMETRY.addLine("RGBA values");
		TELEMETRY.addLine("Right");
		TELEMETRY.addData("Red: ", this.rightRGBAValues.get(0));
		TELEMETRY.addData("Green: ", this.rightRGBAValues.get(1));
		TELEMETRY.addData("Blue: ", this.rightRGBAValues.get(2));
		TELEMETRY.addData("Alpha: ", this.rightRGBAValues.get(3));

		TELEMETRY.addLine("Left");
		TELEMETRY.addData("Red: ", this.leftRGBAValues.get(0));
		TELEMETRY.addData("Green: ", this.leftRGBAValues.get(1));
		TELEMETRY.addData("Blue: ", this.leftRGBAValues.get(2));
		TELEMETRY.addData("Alpha: ", this.leftRGBAValues.get(3));

		TELEMETRY.addLine();
	}
}
