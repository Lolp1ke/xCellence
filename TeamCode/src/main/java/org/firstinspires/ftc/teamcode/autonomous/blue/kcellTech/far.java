package org.firstinspires.ftc.teamcode.autonomous.blue.kcellTech;

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

@Autonomous(name = "Blue far (KcellTech)", group = "!!!BLUE")
public class far extends LinearOpMode {
	private openCV openCV;
	private SampleMecanumDrive movement;
	private arm arm;
	private hand hand;


	@Override
	public void runOpMode() {
		this.openCV = new openCV(false, this.hardwareMap);
		this.movement = new SampleMecanumDrive(this.hardwareMap);
		this.movement.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.movement.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


		this.arm = new arm(this.hardwareMap);


		this.hand = new hand(this.hardwareMap);
		this.hand.wrist(config.WRIST_GROUND);
		this.hand.claw(config.CLAW.CLOSE);


		while (this.opModeInInit()) {
			this.openCV.telemetry(this.telemetry);
			this.openCV.pipeline.telemetry(this.telemetry);

			this.telemetry.update();
		}


		this.waitForStart();
		if (this.isStopRequested()) return;
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
		this.sleep(7000);
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(0d, -9d, Math.toRadians(-90d)))
			.lineToLinearHeading(new Pose2d(30d, -9d, Math.toRadians(-90d)))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(5d, -10d, Math.toRadians(90d)))
			.lineToLinearHeading(new Pose2d(5d, 60d, Math.toRadians(90d)))
			.addTemporalMarker(() -> this.arm.move(45))
			.lineToLinearHeading(new Pose2d(44d, 74d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.hand.wrist(config.WRIST.GROUND);
		this.sleep(700);
		this.hand.claw(config.CLAW.OPEN);
		this.sleep(300);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(5d, 70d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
				.lineToLinearHeading(new Pose2d(5d, 84d, Math.toRadians(90d)))
				.build()
		);
	}

	private void center() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(0d, -9d, 0d))
			.lineToLinearHeading(new Pose2d(30d, -9d, 0d))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(5d, -7d, Math.toRadians(90d)))
//			.lineToLinearHeading(new Pose2d(5d, 60d, Math.toRadians(90d)))
//			.addTemporalMarker(() -> this.arm.move(45))
//			.lineToLinearHeading(new Pose2d(34d, 75d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);


		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(5d, 70d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
				.lineToLinearHeading(new Pose2d(5d, 84d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.GROUND))
				.addTemporalMarker(() -> this.hand.claw(config.CLAW.OPEN))
				.lineToLinearHeading(new Pose2d(5d, 80d, Math.toRadians(90d)))
				.build()
		);
	}

	private void left() {
		final TrajectorySequence backdrop = this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
			.lineToLinearHeading(new Pose2d(0d, -9d, 0d))
			.lineToLinearHeading(new Pose2d(30d, 2d, 0d))
			.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
			.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
			.lineToLinearHeading(new Pose2d(5d, -7d, Math.toRadians(90d)))
//			.lineToLinearHeading(new Pose2d(5d, 60d, Math.toRadians(90d)))
//			.addTemporalMarker(() -> this.arm.move(45))
//			.lineToLinearHeading(new Pose2d(34d, 75d, Math.toRadians(90d)))
			.build();
		this.movement.followTrajectorySequence(backdrop);

		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(backdrop.end())
				.lineToLinearHeading(new Pose2d(5d, 70d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.SCORE))
				.lineToLinearHeading(new Pose2d(5d, 84d, Math.toRadians(90d)))
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.GROUND))
				.addTemporalMarker(() -> this.hand.claw(config.CLAW.OPEN))
				.lineToLinearHeading(new Pose2d(5d, 80d, Math.toRadians(90d)))
				.build()
		);
	}
}
