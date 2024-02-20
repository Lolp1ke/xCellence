package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class imuUtil {
	private final IMU imu;

	public imuUtil(final IMU IMU) {
		this.imu = IMU;
	}

	public imuUtil(final HardwareMap HARDWARE_MAP) {
		this.imu = HARDWARE_MAP.get(IMU.class, "imu");
	}

//	public void init(final HardwareMap hardwareMap, String deviceName) {
//		this.imu = hardwareMap.get(IMU.class, deviceName);
//	}

	public double get() {
		return this.get(AngleUnit.DEGREES);
	}

	public double get(final AngleUnit angleUnit) {
		return this.imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
	}

	public void reset() {
		this.imu.resetYaw();
	}

	public void initialize(final RevHubOrientationOnRobot.LogoFacingDirection logo,
		final RevHubOrientationOnRobot.UsbFacingDirection usb) {
		this.imu.initialize(
			new IMU.Parameters(
				new RevHubOrientationOnRobot(
					logo,
					usb
				)
			)
		);
	}
}
