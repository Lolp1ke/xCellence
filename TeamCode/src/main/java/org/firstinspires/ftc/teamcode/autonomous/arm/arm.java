package org.firstinspires.ftc.teamcode.autonomous.arm;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.motorUtil;
import org.firstinspires.ftc.teamcode.utils.pid;

public class arm extends config {
	private volatile motorUtil motorUtil;
	private volatile DcMotorEx lift;

	private volatile int angle = 0;

	private volatile pid pid;

//	@Override
//	public void run() {
//		this.move(this.angle);
//	}
//
//	public void setAngle(final int ANGLE) {
//		this.angle = ANGLE;
//	}


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


		this.lift = HARDWARE_MAP.get(DcMotorEx.class, "lift");
		this.lift.setDirection(DcMotorEx.Direction.FORWARD);
		this.lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		this.lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


		this.pid = new pid(
			0.1d,
			0d,
			0d,
			0.3d,
			5
		);
	}

	public void move(final int ANGLE) {
		final int TARGET = -(int) (ANGLE * COUNTS_PER_ANGLE);

		this.motorUtil.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		this.motorUtil.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		this.motorUtil.setTargetPosition(TARGET);
		this.motorUtil.setPower(ARM_POWER);

		while (this.motorUtil.isBusy()) ;

		this.motorUtil.setPower(0);
	}


	public void extend(final int TARGET) {
		this.lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		this.lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

		this.lift.setTargetPosition(TARGET);
		this.lift.setPower(LIFT_POWER);

		while (this.lift.isBusy()) ;

		this.lift.setPower(0d);
	}
}
