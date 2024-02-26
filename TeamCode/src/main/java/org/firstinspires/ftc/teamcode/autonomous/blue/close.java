package org.firstinspires.ftc.teamcode.autonomous.blue;

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

@Autonomous(name = "Blue close", group = "!!!BLUE")
public class close extends LinearOpMode {
	private openCV openCV;
	private SampleMecanumDrive movement;
	private volatile arm arm;
	private hand hand;


	@Override
	public void runOpMode() {
		this.openCV = new openCV(false, this.hardwareMap);


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
				this.leftTest();
				break;
			case 2:
				this.centerTest();
				break;
			case 3:
				this.rightTest();
				break;
		}

//		this.leftTest();
	}

	private void rightTest() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(32d, -2d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.addTemporalMarker(() -> this.arm.move(50))
			.lineToLinearHeading(new Pose2d(33d, 38d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.hand.wrist(config.WRIST.GROUND);
		this.sleep(700);
		this.hand.claw(config.CLAW.OPEN);
		this.sleep(300);

		final TrajectorySequence stack = this.movement.trajectorySequenceBuilder(backdrop.end())
			.lineToLinearHeading(new Pose2d(53d, 5d, Math.toRadians(-90d)))
			.lineToLinearHeading(new Pose2d(50d, -75d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.claw(config.CLAW.CLOSE))
			.lineToLinearHeading(new Pose2d(52d, -73d, 0d))
			.lineToLinearHeading(new Pose2d(57d, 0d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.addTemporalMarker(() -> this.arm.move(50))
			.lineToLinearHeading(new Pose2d(27d, 44d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(stack);

		this.arm.move(-5);
		this.hand.wrist(config.WRIST.GROUND);
		this.sleep(700);
		this.hand.claw(config.CLAW.OPEN);
		this.sleep(300);


		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(stack.end())
				.lineToLinearHeading(new Pose2d(20d, 43d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
				.lineToLinearHeading(new Pose2d(51d, 36d, Math.toRadians(90d)))
				.lineToLinearHeading(new Pose2d(51d, 48d, Math.toRadians(90d)))
				.build()
		);
	}


	private void centerTest() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(26d, -5d, 0d))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.lineToLinearHeading(new Pose2d(23d, -5d, 0d))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.addTemporalMarker(() -> this.arm.move(45))
			.lineToLinearHeading(new Pose2d(30d, 43d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.hand.wrist(config.WRIST.GROUND);
		this.sleep(700);
		this.hand.claw(config.CLAW.OPEN);
		this.sleep(300);

		final TrajectorySequence stack = this.movement.trajectorySequenceBuilder(backdrop.end())
			.lineToLinearHeading(new Pose2d(55d, 10d, Math.toRadians(-90d)))
			.lineToLinearHeading(new Pose2d(55d, -70d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.claw(config.CLAW.CLOSE))
			.lineToLinearHeading(new Pose2d(54d, -67d, 0d))
			.lineToLinearHeading(new Pose2d(60d, 0d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.addTemporalMarker(() -> this.arm.move(45))
			.lineToLinearHeading(new Pose2d(39.5d, 50d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(stack);

		this.hand.wrist(config.WRIST.GROUND);
		this.sleep(700);
		this.hand.claw(config.CLAW.OPEN);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(stack.end())
				.lineToLinearHeading(new Pose2d(58d, 40d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
				.lineToLinearHeading(new Pose2d(59d, 50d, Math.toRadians(90d)))
				.build()
		);
	}

	private void leftTest() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(33d, 18d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.lineToLinearHeading(new Pose2d(33d, 21d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.addTemporalMarker(() -> this.arm.move(45))
			.lineToLinearHeading(new Pose2d(21d, 38.5d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.hand.wrist(config.WRIST.GROUND);
		this.sleep(700);
		this.hand.claw(config.CLAW.OPEN);
		this.sleep(300);

		final TrajectorySequence stack = this.movement.trajectorySequenceBuilder(backdrop.end())
			.lineToLinearHeading(new Pose2d(53d, 10d, Math.toRadians(-90d)))
			.lineToLinearHeading(new Pose2d(54d, -74d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.arm.move(-4))
			.addTemporalMarker(() -> this.hand.claw(config.CLAW.CLOSE))
			.lineToLinearHeading(new Pose2d(54d, -72d, 0d))
			.lineToLinearHeading(new Pose2d(57d, 10d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.addTemporalMarker(() -> this.arm.move(45))
			.lineToLinearHeading(new Pose2d(35d, 45.5d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(stack);

		this.hand.wrist(config.WRIST.GROUND);
		this.sleep(700);
		this.hand.claw(config.CLAW.OPEN);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(stack.end())
				.lineToLinearHeading(new Pose2d(55d, 36d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
				.lineToLinearHeading(new Pose2d(55d, 53d, Math.toRadians(90d)))
				.build()
		);
	}

	private void right() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(30d, 15d, 0d))
			.lineToLinearHeading(new Pose2d(29d, 0d, Math.toRadians(-110d)))
			.lineToLinearHeading(new Pose2d(33d, -3d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(41d, 29d, Math.toRadians(-90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.arm.move(163);
		this.sleep(300);
		this.hand.claw(config.CLAW.OPEN);
		this.arm.move(-135);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(54d, 23d, Math.toRadians(-90d)))
				.lineToLinearHeading(new Pose2d(57d, 38d, 0))
				.build()
		);
	}

	private void center() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(28d, 0d, 0d))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.lineToLinearHeading(new Pose2d(25d, 0d, 0d))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(28.5d, 30.3d, Math.toRadians(-90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.arm.move(165);
		this.sleep(300);
		this.hand.claw(false, config.CLAW.OPEN);
		this.arm.move(-135);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(47d, 28d, Math.toRadians(-90d)))
				.lineToLinearHeading(new Pose2d(53d, 45d, 0))
				.build()
		);
	}

	private void left() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(35d, 20d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(0.45))
			.lineToLinearHeading(new Pose2d(27d, 32d, Math.toRadians(-90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.arm.move(160);
		this.sleep(300);
		this.hand.claw(false, config.CLAW.OPEN);
		this.arm.move(-135);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(53d, 28d, Math.toRadians(-90d)))
				.lineToLinearHeading(new Pose2d(53d, 43d, Math.toRadians(0d)))
				.build()
		);
	}
}
