package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class movement2wd {
    private final LinearOpMode opMode;
    private final config _config = new config();

    private DcMotor rightDrive = null;
    private DcMotor leftDrive = null;

    public movement2wd(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void runWithPID() {
    }

    public void run() {
        Gamepad gamepad = this.opMode.gamepad1;
        Telemetry telemetry = this.opMode.telemetry;
        boolean isBoosted = gamepad.right_bumper;

        double rightDrivePower = gamepad.right_stick_x;
        double leftDrivePower = gamepad.left_stick_x;

        this.rightDrive.setPower(rightDrivePower * (isBoosted ? this._config.BOOST : this._config.SPEED));
        this.leftDrive.setPower(leftDrivePower * (isBoosted ? this._config.BOOST : this._config.SPEED));

        telemetry.addData("Right", "%4.2f", rightDrivePower);
        telemetry.addData("Left", "%4.2f", leftDrivePower);
        telemetry.update();
    }

    public void init() {
        this.rightDrive = this.opMode.hardwareMap.get(DcMotor.class, "right_drive");
        this.leftDrive = this.opMode.hardwareMap.get(DcMotor.class, "left_drive");

        this.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
