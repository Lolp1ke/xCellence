package org.firstinspires.ftc.teamcode.main;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.main.arm.arm;
import org.firstinspires.ftc.teamcode.main.hand.hand;
import org.firstinspires.ftc.teamcode.main.movement.movement;
import org.firstinspires.ftc.teamcode.main.rocket.rocket;
import org.firstinspires.ftc.teamcode.main.touch.touch;

@TeleOp(name = "main", group = "!xCellence")
public class main extends LinearOpMode {
	private movement movement;
	private arm arm;
	private hand hand;
	private rocket rocket;
	private touch touch;

	private Telemetry telemetryGraph;

	@Override
	public void runOpMode() {
		this.movement = new movement(this.hardwareMap);
		this.arm = new arm(this.hardwareMap);
		this.hand = new hand(this.hardwareMap);
		this.rocket = new rocket(this.hardwareMap);
		this.touch = new touch(this.hardwareMap);


		this.telemetryGraph = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


		this.waitForStart();
		while (this.opModeIsActive() && !this.isStopRequested()) {
			this.movement.run(this.gamepad1);
			this.arm.run(this.gamepad2);
			this.hand.run(this.gamepad2);
			this.rocket.run(this.gamepad1);
			this.touch.run();


			this.movement.telemetry(this.telemetryGraph);
			this.arm.telemetry(this.telemetryGraph);
			this.hand.telemetry(this.telemetry);
			this.rocket.telemetry(this.telemetry);
			this.touch.telemetry(this.telemetry);
			this.telemetry.update();
		}
	}
}
