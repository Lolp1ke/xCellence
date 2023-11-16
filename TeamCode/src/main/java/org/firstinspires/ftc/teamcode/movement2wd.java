package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Disabled
public class movement2wd {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private final config _config = new config();

    private final LinearOpMode opmode;

    public movement2wd(LinearOpMode _opmode) {
        opmode = _opmode;
    }

    public void car() {
        double leftPower;
        double rightPower;

        double drive = -opmode.gamepad1.left_stick_y;
        double turn = opmode.gamepad1.right_stick_x;
        
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);

        leftDrive.setPower(leftPower * _config.SPEED);
        rightDrive.setPower(rightPower * _config.SPEED);

        opmode.telemetry.addData("Left: ", leftPower);
        opmode.telemetry.addData("Right: ", rightDrive);
    }


    public void tank() {
        double leftPower = -opmode.gamepad1.left_stick_y;
        double rightPower = opmode.gamepad1.right_stick_x;

        leftPower = Range.clip(leftPower, -1.0, 1.0);
        rightPower = Range.clip(rightPower, -1.0, 1.0);

        leftDrive.setPower(leftPower * _config.SPEED);
        rightDrive.setPower(rightPower * _config.SPEED);

        opmode.telemetry.addData("Left: ", leftPower);
        opmode.telemetry.addData("Right: ", rightDrive);
    }

    public void init() {
        rightDrive = opmode.hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive = opmode.hardwareMap.get(DcMotor.class, "left_drive");

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
