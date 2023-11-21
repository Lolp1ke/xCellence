package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class mechanism {
    private final LinearOpMode opmode;
    private final config _config = new config();

    private DcMotor right_arm;
    private DcMotor left_arm;

    private Servo hand;
    private Servo claw;

    private double handPosition = 1.0d;

    public mechanism(LinearOpMode _opMode) {
        opmode = _opMode;
    }

    public void run() {
        double armPower = opmode.gamepad2.left_stick_y * _config.ARM_SPEED;
        double avgPosition = (right_arm.getCurrentPosition() + left_arm.getCurrentPosition()) / 2.0d;

        if (opmode.gamepad2.x) {
            handPosition = 1.0d;
        } else if (opmode.gamepad2.a) {
            handPosition = 0.3d;
        } else if (opmode.gamepad2.b) {
            handPosition = 0.0d;
        }

        if (opmode.gamepad2.right_bumper) {
            armPower = 0.1d;
        } else if (opmode.gamepad2.left_bumper) {
            armPower = -0.1d;
        }

        if (avgPosition > -30) {
            right_arm.setPower(armPower);
            left_arm.setPower(armPower);
        } else {
            right_arm.setPower(0); // + is up?
            left_arm.setPower(0);
        }

        hand.setPosition(handPosition);
        claw.setPosition(opmode.gamepad2.y ? 0.8d : 0.6d);

        opmode.telemetry.addData("Arm: ", armPower);
        opmode.telemetry.addData("Hand: ", handPosition);

        opmode.telemetry.addLine("Dev:");
        opmode.telemetry.addData("Average arm position: ", avgPosition);
    }

    public void init() {
        right_arm = opmode.hardwareMap.get(DcMotor.class, "right_arm");
        left_arm = opmode.hardwareMap.get(DcMotor.class, "left_arm");

        hand = opmode.hardwareMap.get(Servo.class, "hand");
        claw = opmode.hardwareMap.get(Servo.class, "claw");

        right_arm.setDirection(DcMotorSimple.Direction.FORWARD);
        left_arm.setDirection(DcMotorSimple.Direction.REVERSE);

        hand.setPosition(handPosition);
        claw.setPosition(0.0d);
    }
}
