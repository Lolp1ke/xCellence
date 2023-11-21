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
        boolean isBoosted = opmode.gamepad1.right_bumper;
        boolean isSlowed = opmode.gamepad1.left_bumper;

        double drive = -opmode.gamepad1.left_stick_y;
        double turn = opmode.gamepad1.right_stick_x;

        double rightPower = Range.clip(drive - turn, -1.0, 1.0);
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);

        double speedMultiplier = isBoosted ? _config.ACCELERATION : (isSlowed ? _config.DECELERATION : _config.SPEED);
        
        rightDrive.setPower(rightPower * speedMultiplier);
        leftDrive.setPower(leftPower * speedMultiplier);

        opmode.telemetry.addData("Left: ", leftPower);
        opmode.telemetry.addData("Right: ", rightPower);
        opmode.telemetry.addData("Boost: ", isBoosted);
    }


    public void tank() {
        boolean isBoosted = opmode.gamepad1.right_bumper;
        boolean isSlowed = opmode.gamepad1.left_bumper;

        double leftPower = -opmode.gamepad1.left_stick_y;
        double rightPower = opmode.gamepad1.right_stick_x;

        double speedMultiplier = isBoosted ? _config.ACCELERATION : (isSlowed ? _config.DECELERATION : _config.SPEED);
        leftDrive.setPower(leftPower * speedMultiplier);
        rightDrive.setPower(rightPower * speedMultiplier);

        opmode.telemetry.addData("Left: ", leftPower);
        opmode.telemetry.addData("Right: ", rightPower);
        opmode.telemetry.addData("Boost: ", isBoosted);
    }

    public void init() {
        rightDrive = opmode.hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive = opmode.hardwareMap.get(DcMotor.class, "left_drive");

        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
    }
}
