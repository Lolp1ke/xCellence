package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Roadrunner", group = "test")
public class roadrunner extends LinearOpMode {
	SampleMecanumDrive drive;

	@Override
	public void runOpMode() {
		drive = new SampleMecanumDrive(hardwareMap);

		Trajectory trajectory = drive.trajectoryBuilder(new Pose2d()).forward(10).build();

		waitForStart();
		if (isStopRequested()) return;

		drive.followTrajectory(trajectory);
	}
}
