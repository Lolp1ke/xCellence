package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.roadrunner.drive.SampleMecanumDrive;

@TeleOp(name = "Positioning", group = "test")
public class autonomousPosition extends LinearOpMode {
	private SampleMecanumDrive movement;

	@Override
	public void runOpMode() {
		this.movement = new SampleMecanumDrive(this.hardwareMap);
		this.movement.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		this.movement.setPoseEstimate(new Pose2d(0d, 0d, 0d));


		this.waitForStart();
		while (this.opModeIsActive()) {
			final Pose2d currentPosition = this.movement.getPoseVelocity();

			this.telemetry.addLine("Position");
//			this.telemetry.addData("X: ", currentPosition.component1());
//			this.telemetry.addData("Y: ", currentPosition.getY());
//			this.telemetry.addData("Heading: ", currentPosition.getHeading());
			this.telemetry.update();
		}
	}
}
