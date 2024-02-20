package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.arm.arm;
import org.firstinspires.ftc.teamcode.autonomous.hand.hand;
import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.sigma.hand.config;

@Autonomous(name = "Red far", group = "!!!RED")
public class far extends LinearOpMode {
	private openCV openCV;
	private SampleMecanumDrive movement;
	private arm arm;
	private hand hand;


	@Override
	public void runOpMode() {
		this.openCV = new openCV(true, this.hardwareMap);


		this.movement = new SampleMecanumDrive(this.hardwareMap);
		this.movement.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.movement.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


		this.arm = new arm(this.hardwareMap);


		this.hand = new hand(this.hardwareMap);


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

		while (this.opModeIsActive()) {
		}
	}

	private void right() {
		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
				.lineToSplineHeading(new Pose2d(28d, 5d, Math.toRadians(-90)))
				//.forward(20)
				.addTemporalMarker(() -> this.hand.wrist(config.WRIST.GROUND))
				.back(10)
				.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
				.addTemporalMarker(() -> this.arm.move(30))
//				.forward()
				.build()

		);
	}

	private void center() {
		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
				.forward(45)
				//.splineToSplineHeading(new Pose2d(), Math.toRadians(90))
				// .lineToSplineHeading(new Pose2d())
				.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.OPEN))
				.back(10)
				.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.CLOSE))
				.splineToSplineHeading(new Pose2d(), Math.toRadians(90))
				.forward(120)
				.addTemporalMarker(() -> this.arm.move(100))
				.forward(15)
				.addTemporalMarker(() -> this.arm.move(-40))
				.addTemporalMarker(() -> this.hand.claw(false, config.CLAW.OPEN))
				.back(15)
				.addTemporalMarker(() -> this.arm.move(-60))
				.addTemporalMarker(() -> this.hand.claw(true, config.CLAW.CLOSE))
				.strafeLeft(60)
				.forward(25)
				.build()
		);
	}

	private void left() {
		this.movement.followTrajectorySequence(
			this.movement.trajectorySequenceBuilder(new Pose2d(0d, 0d, 0d))
				.lineToSplineHeading(new Pose2d(28d, 5d, Math.toRadians(90)))
				.forward(20)
				.lineToSplineHeading(new Pose2d(24d, -90d, Math.toRadians(-90)))
				.addTemporalMarker(() -> this.arm.move(100))
				.back(10)
				.lineToSplineHeading(new Pose2d(45d, -90d, Math.toRadians(0)))
				.build()
		);
	}
}
