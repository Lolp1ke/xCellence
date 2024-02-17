package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class servoUtil {
	private final List<Servo> servos;


	public servoUtil(final Servo... SERVOS) {
		this.servos = Arrays.asList(SERVOS);
	}

//	public servoUtil(final HardwareMap HARDWARE_MAP, final String... DEVICES_NAME) {
//		this.servos = Collections.emptyList();
//
//		for (final String DEVICE_NAME : DEVICES_NAME)
//			this.servos.add(HARDWARE_MAP.get(Servo.class, DEVICE_NAME));
//	}

	public void setDirection(final Servo.Direction... DIRECTIONS) {
		for (int i = 0; i < DIRECTIONS.length; i++)
			this.servos.get(i).setDirection(DIRECTIONS[i]);
	}

	public void setDirection(final Servo.Direction DIRECTION) {
		for (final Servo SERVO : this.servos)
			SERVO.setDirection(DIRECTION);
	}

	public void setPosition(final double... POSITIONS) {
		for (int i = 0; i < POSITIONS.length; i++)
			this.servos.get(i).setPosition(POSITIONS[i]);
	}

	public void setPosition(final double POSITION) {
		for (final Servo servo : this.servos)
			servo.setPosition(POSITION);
	}
}
