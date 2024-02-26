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

@Autonomous(name = "Blue far", group = "!!!BLUE")
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
	}
}
