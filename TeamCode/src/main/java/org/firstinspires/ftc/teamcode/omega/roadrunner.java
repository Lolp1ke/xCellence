package org.firstinspires.ftc.teamcode.omega;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "Roadrunner", group = "test")
public class roadrunner extends LinearOpMode {
	private SampleMecanumDrive drive;

	@Override
	public void runOpMode() {
		drive = new SampleMecanumDrive(hardwareMap);
		drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		drive.setPoseEstimate(new Pose2d(0, 0, 0));

//		TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
//			.build();

//		Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
//			.splineTo(new Vector2d(36, 36), Math.toRadians(0))
//			.build();

		waitForStart();
		if (isStopRequested()) return;

//		drive.followTrajectory(trajectory);

		drive.followTrajectorySequence(
			drive.trajectorySequenceBuilder(new Pose2d())
				.forward(30)
				.splineTo(new Vector2d(10, 10), 0)
				.setReversed(true)
				.splineTo(new Vector2d(0, 0), 0)
				.build()
		);

		while (opModeIsActive()) {
		}
	}
}
