package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class movement2wd {
    private final LinearOpMode opMode;
    private final config _config = new config();
    private DcMotor rightDrive;
    private DcMotor leftDrive;


    private double targetPosition = 10000;
    private double integral = 0;
    private double prevError = 0;

    public movement2wd(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void run() {
        boolean isBoosted = opMode.gamepad1.right_bumper;

        double rightDrivePower = -opMode.gamepad1.right_stick_y * (isBoosted ? _config.BOOST : _config.SPEED);
        double leftDrivePower = -opMode.gamepad1.left_stick_y * (isBoosted ? _config.BOOST : _config.SPEED);

        rightDrive.setPower(rightDrivePower);
        leftDrive.setPower(leftDrivePower);

        opMode.telemetry.addData("Right drive: ", rightDrivePower);
        opMode.telemetry.addData("Left drive: ", leftDrivePower);
        opMode.telemetry.addData("r pos: ", rightDrive.getCurrentPosition());
        opMode.telemetry.addData("l pos: ", leftDrive.getCurrentPosition());
        opMode.telemetry.addData("Boost: ", isBoosted);
        opMode.telemetry.update();
    }

    public void _run() {
        boolean isBoosted = opMode.gamepad1.right_bumper;

        double yPower = -opMode.gamepad1.right_stick_y * (isBoosted ? _config.BOOST : _config.SPEED);
        double xPower = opMode.gamepad1.left_stick_x * (isBoosted ? _config.BOOST : _config.SPEED);

        rightDrive.setPower(yPower - xPower);
        leftDrive.setPower(yPower + xPower);

        opMode.telemetry.addData("y drive: ", yPower);
        opMode.telemetry.addData("x drive: ", xPower);
        opMode.telemetry.addData("r pos: ", rightDrive.getCurrentPosition());
        opMode.telemetry.addData("l pos: ", leftDrive.getCurrentPosition());
        opMode.telemetry.addData("Boost: ", isBoosted);
        opMode.telemetry.update();
    }

    public void init() {
        rightDrive = opMode.hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive = opMode.hardwareMap.get(DcMotor.class, "left_drive");

        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
