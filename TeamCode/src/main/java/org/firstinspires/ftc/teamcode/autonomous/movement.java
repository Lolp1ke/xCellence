package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class movement {
	public final SampleMecanumDrive drive;

	public movement(final HardwareMap hardwareMap) {
		this.drive = new SampleMecanumDrive(hardwareMap);
	}
}
