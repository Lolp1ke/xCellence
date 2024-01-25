package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

public class motorUtil {
	private final List<DcMotorEx> motors;

	public motorUtil(final DcMotorEx... motors) {
		this.motors = Arrays.asList(motors);
	}

	public void init(final HardwareMap HARDWARE_MAP, final String... DEVICE_NAMES) {
		for (int i = 0; i < this.motors.size(); i++) {
			this.motors.set(i, HARDWARE_MAP.get(DcMotorEx.class, DEVICE_NAMES[i]));
		}
	}

	public boolean isBusy() {
		for (final DcMotorEx motor : this.motors) {
			if (!motor.isBusy()) return false;
		}

		return true;
	}

	public void setMode(final DcMotorEx.RunMode runMode) {
		for (final DcMotorEx motor : this.motors) {
			motor.setMode(runMode);
		}
	}

	public void setZeroPowerBehaviour(final DcMotorEx.ZeroPowerBehavior ZERO_POWER_BEHAVIOUR) {
		for (final DcMotorEx motor : this.motors) {
			motor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOUR);
		}
	}

	public void setZeroPowerBehaviour(final DcMotorEx.ZeroPowerBehavior... ZERO_POWER_BEHAVIOURS) {
		for (int i = 0; i < this.motors.size(); i++) {
			this.motors.get(i).setZeroPowerBehavior(ZERO_POWER_BEHAVIOURS[i]);
		}
	}

	public void setPower(final double POWER) {
		for (final DcMotorEx motor : this.motors) {
			motor.setPower(POWER);
		}
	}

	public void setPower(final double... POWERS) {
		for (int i = 0; i < this.motors.size(); i++) {
			this.motors.get(i).setPower(POWERS[i]);
		}
	}

	public void setDirection(final DcMotorSimple.Direction DIRECTION) {
		for (final DcMotorEx motor : this.motors) {
			motor.setDirection(DIRECTION);
		}
	}

	public void setDirection(final DcMotorSimple.Direction... DIRECTIONS) {
		for (int i = 0; i < this.motors.size(); i++) {
			this.motors.get(i).setDirection(DIRECTIONS[i]);
		}
	}

	public void setTargetPosition(final int TARGET) {
		for (final DcMotorEx motor : this.motors) {
			motor.setTargetPosition(TARGET);
		}
	}

	public void setTargetPosition(final int... TARGETS) {
		for (int i = 0; i < this.motors.size(); i++) {
			this.motors.get(i).setTargetPosition(TARGETS[i]);
		}
	}

	public void setTargetPositionToTolerance(final int TOLERANCE) {
		for (final DcMotorEx motor : this.motors) {
			motor.setTargetPositionTolerance(TOLERANCE);
		}
	}

	public void setTargetPositionToTolerance(final int... TOLERANCES) {
		for (int i = 0; i < this.motors.size(); i++) {
			this.motors.get(i).setTargetPositionTolerance(TOLERANCES[i]);
		}
	}

	public void setVelocity(final double ANGULAR_RATE) {
		for (final DcMotorEx motor : this.motors) {
			motor.setVelocity(ANGULAR_RATE);
		}
	}

	public void setVelocity(final double ANGULAR_RATE, final AngleUnit ANGLE_UNIT) {
		for (final DcMotorEx motor : this.motors) {
			motor.setVelocity(ANGULAR_RATE, ANGLE_UNIT);
		}
	}

	public HashMap<Integer, Integer> getCurrentPositions() {
		HashMap<Integer, Integer> currentPositions = new HashMap<>();

		for (final DcMotorEx motor : this.motors) {
			currentPositions.put(motor.getPortNumber(), motor.getCurrentPosition());
		}

		return currentPositions;
	}
}
