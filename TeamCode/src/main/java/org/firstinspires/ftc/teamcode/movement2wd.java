package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class movement2wd {
    private final LinearOpMode opMode;
    private final config _config;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    private DcMotor rightDrive;
    private DcMotor leftDrive;

    public movement2wd(LinearOpMode opMode) {
        this.opMode = opMode;
        this._config = new config();
        this.gamepad = this.opMode.gamepad1;
        this.telemetry = this.opMode.telemetry;
    }

    public void run() {
        boolean isBoosted = this.gamepad.right_bumper;
        double rightDrivePower = this.gamepad.right_stick_y * (isBoosted ? this._config.BOOST : this._config.SPEED);
        double leftDrivePower = this.gamepad.left_stick_y * (isBoosted ? this._config.BOOST : this._config.SPEED);

        this.rightDrive.setPower(rightDrivePower);
        this.leftDrive.setPower(leftDrivePower);

        this.telemetry.addData("Right drive: ", rightDrivePower);
        this.telemetry.addData("Left drive: ", leftDrivePower);
        this.telemetry.update();
    }
    public void init() {
        this.rightDrive = this.opMode.hardwareMap.get(DcMotor.class, "right_drive");
        this.leftDrive = this.opMode.hardwareMap.get(DcMotor.class, "left_device");

        this.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        this.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
