package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class movement2wd {
    private final LinearOpMode opMode;
    private final config _config = new config();
    private DcMotor rightDrive;
    private DcMotor leftDrive;


    private double targetPosition = 0;
    private double integral = 0;
    private double prevError = 0;

    public movement2wd(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void runWithPID() {
        double rightDrivePower = -this.opMode.gamepad1.right_stick_y;
        double leftDrivePower = -this.opMode.gamepad1.left_stick_y;

        double avgPosition = (this.rightDrive.getCurrentPosition() + this.leftDrive.getCurrentPosition()) / 2.0;

        double error = this.targetPosition - avgPosition;

        double proportional = this._config.kP * error;
        this.integral += this._config.kI * error;
        double derivative = this._config.kD * (error - this.prevError);

        double output = proportional + this.integral + derivative;

        rightDrivePower = Range.clip(rightDrivePower + output, -1, 1);
        leftDrivePower = Range.clip(leftDrivePower + output, -1, 1);

        this.rightDrive.setPower(rightDrivePower);
        this.leftDrive.setPower(leftDrivePower);

        this.prevError = error;
        this.opMode.telemetry.addData("Right: ", rightDrivePower);
        this.opMode.telemetry.addData("Left: ", leftDrivePower);
        this.opMode.telemetry.update();
    }

    public void run() {
        boolean isBoosted = this.opMode.gamepad1.right_bumper;

        double rightDrivePower = -this.opMode.gamepad1.right_stick_y * (isBoosted ? this._config.BOOST : this._config.SPEED);
        double leftDrivePower = -this.opMode.gamepad1.left_stick_y * (isBoosted ? this._config.BOOST : this._config.SPEED);

        this.rightDrive.setPower(rightDrivePower);
        this.leftDrive.setPower(leftDrivePower);

        this.opMode.telemetry.addData("Right drive: ", rightDrivePower);
        this.opMode.telemetry.addData("Left drive: ", leftDrivePower);
        this.opMode.telemetry.addData("Boost: ", isBoosted);
        this.opMode.telemetry.update();
    }

    public void init() {
        this.rightDrive = this.opMode.hardwareMap.get(DcMotor.class, "right_drive");
        this.leftDrive = this.opMode.hardwareMap.get(DcMotor.class, "left_drive");

        this.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
