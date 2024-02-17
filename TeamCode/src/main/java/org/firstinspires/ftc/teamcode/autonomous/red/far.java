package org.firstinspires.ftc.teamcode.autonomous.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.openCV.openCV;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

public class far extends LinearOpMode {
	private final openCV openCV = new openCV(true);
	private final SampleMecanumDrive movement;

	private far() {
		this.movement = new SampleMecanumDrive(this.hardwareMap);
		this.movement.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		this.movement.setPoseEstimate(new Pose2d());
	}

	@Override
	public void runOpMode() {
		this.openCV.init(this.hardwareMap);

		while (opModeInInit()) {
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
			this.movement.trajectorySequenceBuilder(new Pose2d())
				.forward(30)
				.splineTo(new Vector2d(10, 10), 0)
				.setReversed(true)
				.splineTo(new Vector2d(0, 0), 0)
				.build()
		);
	}

	private void center() {
	}

	private void left() {
	}
}
