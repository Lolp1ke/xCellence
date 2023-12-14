package org.firstinspires.ftc.teamcode.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;

@Autonomous(name = "Red close", group = "!Red")
public class close extends LinearOpMode {
	private final openCV _openCV = new openCV(this, true);
	
	@Override
	public void runOpMode() {
	}
}
