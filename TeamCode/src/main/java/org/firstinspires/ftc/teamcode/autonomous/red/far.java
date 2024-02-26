package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.arm.arm;
import org.firstinspires.ftc.teamcode.autonomous.hand.hand;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;
import org.firstinspires.ftc.teamcode.main.hand.config;
import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.utils.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red far", group = "!!!RED")
public class far extends LinearOpMode {
	private openCV openCV;
	private SampleMecanumDrive movement;
	private volatile arm arm;
	private hand hand;


	@Override
	public void runOpMode() {
		this.openCV = new openCV(true, this.hardwareMap);


		this.movement = new SampleMecanumDrive(this.hardwareMap);
		this.movement.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.movement.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


		this.arm = new arm(this.hardwareMap);


		this.hand = new hand(this.hardwareMap);
		this.hand.wrist(config.WRIST.GROUND);
		this.hand.claw(config.CLAW.CLOSE);


		while (this.opModeInInit()) {
			this.openCV.telemetry(this.telemetry);
			this.openCV.pipeline.telemetry(this.telemetry);

			this.telemetry.update();
		}

		this.waitForStart();
		this.openCV.cameraOff(this.telemetry);
		final int location = this.openCV.pipeline.location;

		switch (location) {
			case 1:
				this.left();
				break;

			case 2:
				this.center();
				break;

			default:
				this.right();
				break;
		}

		this.right();
	}

	private void right() {
//		this.movement.followTrajectorySequence(
//			this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
//				.lineToSplineHeading(new Pose2d(28d, 5d, Math.toRadians(-90d)))
//				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.GROUND))
//				.back(10)
//				.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
//				.lineToSplineHeading(new Pose2d(28d, -8d, Math.toRadians(90d)))
//				.build()
//		);

		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToSplineHeading(new Pose2d(28d, 0d, Math.toRadians(90d)))
			.lineToLinearHeading(new Pose2d(28d, -25d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(28d, -80d, Math.toRadians(90d)))
			.build();

		this.movement.followTrajectorySequence(backdrop);
	}

	private void center() {
		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
				.lineToLinearHeading(new Pose2d(28d, 0d, 0d))
				.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
				.lineToSplineHeading(new Pose2d(20d, 0d, 0d))
				.lineToSplineHeading(new Pose2d(30d, 15d, Math.toRadians(90d)))
				.addTemporalMarker(() -> {
					this.arm.move(13);
//					this.arm.setAngle(13);
//					this.arm.start();
				})
				.lineToSplineHeading(new Pose2d(30d, 28d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.CLOSE))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
				.lineToSplineHeading(new Pose2d(25d, -50d, Math.toRadians(-90d)))
				.addTemporalMarker(() -> {
					this.arm.move(40);
//					this.arm.setAngle(40);
//					this.arm.start();
				})
				.lineToSplineHeading(new Pose2d(25d, -83d, Math.toRadians(-90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.MID))
//				.lineToSplineHeading(new Pose2d(25d, -92d, Math.toRadians(-90d)))
				.addTemporalMarker(() -> this.hand.claw(config.CLAW.OPEN))
				.build()
		);
	}

	private void left() {
		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
				.lineToSplineHeading(new Pose2d(28d, 5d, Math.toRadians(90d)))
				.forward(20d)
				.lineToSplineHeading(new Pose2d(24d, -90d, Math.toRadians(-90d)))
				.addTemporalMarker(() -> this.arm.move(100))
				.back(10d)
				.lineToSplineHeading(new Pose2d(45d, -90d, Math.toRadians(0d)))
				.build()
		);
	}
}
