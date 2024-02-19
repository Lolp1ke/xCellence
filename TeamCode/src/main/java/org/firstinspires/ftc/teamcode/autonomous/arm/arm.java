package org.firstinspires.ftc.teamcode.autonomous.arm;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.pid;

public class arm extends config {
	private final motorUtil motorUtil;

	private final pid pid;


	public arm(final HardwareMap HARDWARE_MAP) {
		this.motorUtil = new motorUtil(
			HARDWARE_MAP.get(DcMotorEx.class, "right_arm"),
			HARDWARE_MAP.get(DcMotorEx.class, "left_arm")
		);

		this.motorUtil.setDirection(
			DcMotorEx.Direction.FORWARD,
			DcMotorEx.Direction.REVERSE
		);

		this.motorUtil.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		this.motorUtil.setZeroPowerBehaviour(DcMotorEx.ZeroPowerBehavior.BRAKE);


		this.pid = new pid(
			0.02d,
			0d,
			0d,
			0.3d
		);
	}

	public void move(final int angle) {
		final int target = -(int) (angle * COUNTS_PER_ANGLE);

		this.motorUtil.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		this.motorUtil.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		this.motorUtil.setTargetPosition(target);
		this.motorUtil.setPower(ARM_POWER);

		while (this.motorUtil.isBusy()) ;

		this.motorUtil.setPower(0);
	}

}
