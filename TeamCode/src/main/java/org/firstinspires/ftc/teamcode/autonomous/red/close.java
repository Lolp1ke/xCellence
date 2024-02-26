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

@Autonomous(name = "Red close", group = "!!!RED")
public class close extends LinearOpMode {
	private openCV openCV;

	private SampleMecanumDrive movement;
	private arm arm;
	private hand hand;

	@Override
	public void runOpMode() {
		this.openCV = new openCV(true, this.hardwareMap);


		this.movement = new SampleMecanumDrive(this.hardwareMap);
		this.movement.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.movement.setPoseEstimate(new Pose2d(0, 0, 0));


		this.arm = new arm(this.hardwareMap);


		this.hand = new hand(this.hardwareMap);
		this.hand.claw(config.CLAW.CLOSE);
		this.hand.wrist(config.WRIST.GROUND);


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
	}

	private void right() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(35d, -20d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(0.45))
			.lineToLinearHeading(new Pose2d(18d, -28d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.arm.move(160);
		this.sleep(300);
		this.hand.claw(false, config.CLAW.OPEN);
		this.arm.move(-135);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(47d, -24d, Math.toRadians(90d)))
				.lineToLinearHeading(new Pose2d(47d, -43d, Math.toRadians(0d)))
				.build()
		);
//		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
//			.lineToLinearHeading(new Pose2d(30d, -25d, Math.toRadians(90d)))
//			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
//			.addTemporalMarker(() -> this.hand.wrist(0.45))
//			.lineToLinearHeading(new Pose2d(16d, -36d, Math.toRadians(90d)))
//			.build();
//		this.movement.followTrajectorySequence(backdrop);
//
//		this.arm.move(160);
//		this.sleep(300);
//		this.hand.claw(false, config.CLAW.OPEN);
//		this.arm.move(-135);
//		this.sleep(300);

		final TrajectorySequence stack = this.movement.trajectorySequenceBuilder(backdrop.end())
			.lineToLinearHeading(new Pose2d(26d, -15d, Math.toRadians(90d)))
			.lineToLinearHeading(new Pose2d(24d, 60d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.GROUND))
			.lineToLinearHeading(new Pose2d(24d, 68.5d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.claw(config.CLAW.CLOSE))
			.lineToLinearHeading(new Pose2d(22d, 67.5d, 0d))
			.lineToLinearHeading(new Pose2d(24d, 68.5d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(29d, -15d, Math.toRadians(90d)))
			.lineToLinearHeading(new Pose2d(30d, -39d, Math.toRadians(90d)))
			.build();
//		this.movement.followTrajectorySequence(stack);
//
//		this.arm.move(160);
//		this.sleep(300);
//		this.hand.claw(false, config.CLAW.OPEN);
//		this.arm.move(-135);
//		this.sleep(300);

//		this.movement.followTrajectorySequence(
//			this.movement.trajectorySequenceBuilder(backdrop.end())
//				.lineToLinearHeading(new Pose2d(50d, -43d, Math.toRadians(90d)))
//				.build()
//		);
	}

	private void center() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(29d, 0d, 0d))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.lineToLinearHeading(new Pose2d(25d, 0d, 0d))
			.addTemporalMarker(() -> this.hand.wrist(0.44))
			.lineToLinearHeading(new Pose2d(24d, -28d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.arm.move(165);
		this.sleep(300);
		this.hand.claw(false, config.CLAW.OPEN);
		this.arm.move(-135);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(47d, -35d, Math.toRadians(90d)))
				.lineToLinearHeading(new Pose2d(50d, -45d, 0))
				.build()
		);
//		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
//			.lineToLinearHeading(new Pose2d(29d, 0d, 0d))
//			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
//			.lineToLinearHeading(new Pose2d(25d, 0d, 0d))
//			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
//			.lineToLinearHeading(new Pose2d(22d, -36d, Math.toRadians(90d)))
//			.build();
//		this.movement.followTrajectorySequence(backdrop);
//
//		this.arm.move(165);
//		this.sleep(300);
//		this.hand.claw(false, config.CLAW.OPEN);
//		this.arm.move(-135);
//		this.sleep(300);

		final TrajectorySequence stack = this.movement.trajectorySequenceBuilder(backdrop.end())
			.lineToLinearHeading(new Pose2d(28d, 60d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.GROUND))
			.lineToLinearHeading(new Pose2d(27d, 71.5d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.claw(config.CLAW.CLOSE))
			.lineToLinearHeading(new Pose2d(21d, 66d, 0))
			.lineToLinearHeading(new Pose2d(24d, 68.5d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(25d, -39d, Math.toRadians(90d)))
			.build();
//		this.movement.followTrajectorySequence(stack);
//
//		this.arm.move(160);
//		this.sleep(300);
//		this.hand.claw(config.CLAW.OPEN);
//		this.arm.move(-135);
//		this.sleep(300);

//		this.movement.followTrajectorySequence(
//			this.movement.trajectorySequenceBuilder(backdrop.end())
//				.lineToLinearHeading(new Pose2d(47d, -35d, Math.toRadians(90d)))
//				.lineToLinearHeading(new Pose2d(50d, -50d, 0))
//				.build()
//		);
	}

	private void left() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(30d, -15d, 0d))
			.lineToLinearHeading(new Pose2d(29d, 0d, Math.toRadians(110d)))
			.lineToLinearHeading(new Pose2d(33d, 8d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(34.5d, -29d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.arm.move(163);
		this.sleep(300);
		this.hand.claw(config.CLAW.OPEN);
		this.arm.move(-135);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(54d, -23d, Math.toRadians(90d)))
				.lineToLinearHeading(new Pose2d(57d, -38d, 0))
				.build()
		);
//		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
//			.lineToLinearHeading(new Pose2d(30d, -15d, 0d))
//			.lineToLinearHeading(new Pose2d(32d, 0d, Math.toRadians(110d)))
//			.lineToLinearHeading(new Pose2d(33d, 0d, Math.toRadians(90d)))
//			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
//			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
//			.lineToLinearHeading(new Pose2d(33d, -34d, Math.toRadians(90d)))
//			.build();
//		this.movement.followTrajectorySequence(backdrop);
//
//		this.arm.move(163);
//		this.sleep(300);
//		this.hand.claw(config.CLAW.OPEN);
//		this.arm.move(-135);
//		this.sleep(300);

		final TrajectorySequence stack = this.movement.trajectorySequenceBuilder(backdrop.end())
			.lineToLinearHeading(new Pose2d(26d, -5d, Math.toRadians(90d)))
			.lineToLinearHeading(new Pose2d(27d, 60d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.GROUND))
			.lineToLinearHeading(new Pose2d(27d, 71.5d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.claw(config.CLAW.CLOSE))
			.lineToLinearHeading(new Pose2d(21d, 66d, 0))
			.lineToLinearHeading(new Pose2d(28d, 68.5d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(28d, -37.5d, Math.toRadians(90d)))
			.build();
//		this.movement.followTrajectorySequence(stack);

//		this.arm.move(160);
//		this.sleep(300);
//		this.hand.claw(config.CLAW.OPEN);
//		this.arm.move(-135);
//		this.sleep(300);

//		this.movement.followTrajectorySequence(
//			this.movement.trajectorySequenceBuilder(backdrop.end())
//				.lineToLinearHeading(new Pose2d(50d, -39d, Math.toRadians(90d)))
//				.lineToLinearHeading(new Pose2d(50d, -50d, 0))
//				.build()
//		);
	}
}
