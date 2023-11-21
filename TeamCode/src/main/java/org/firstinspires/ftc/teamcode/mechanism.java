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

    private Servo right_claw;
    private Servo left_claw;

    private double handPosition = _config.HAND_GROUND;

    public mechanism(LinearOpMode _opMode) {
        opmode = _opMode;
    }

    public void run() {
        double armPower = opmode.gamepad2.left_stick_y * _config.ARM_SPEED;
        double avgPosition = (right_arm.getCurrentPosition() + left_arm.getCurrentPosition()) / 2.0d;

        if (opmode.gamepad2.x) {
            handPosition = _config.HAND_CLOSE;
        } else if (opmode.gamepad2.a) {
            handPosition = _config.HAND_GROUND;
        }

        if (opmode.gamepad2.right_bumper) {
            armPower = 0.1d;
        } else if (opmode.gamepad2.left_bumper) {
            armPower = -0.1d;
        }

//        if (avgPosition < -5) {
        right_arm.setPower(armPower);
        left_arm.setPower(armPower);
//        } else {
//            right_arm.setPower(-0.3d);
//            left_arm.setPower(-0.3d);
//        }

        hand.setPosition(handPosition);
        right_claw.setPosition(opmode.gamepad2.y ? _config.CLAW_CLOSE : opmode.gamepad2.dpad_right ? _config.CLAW_CLOSE : _config.CLAW_OPEN);
        left_claw.setPosition(opmode.gamepad2.y ? _config.CLAW_CLOSE : opmode.gamepad2.dpad_left ? _config.CLAW_CLOSE : _config.CLAW_OPEN);

//        right_claw.setPosition(opmode.gamepad2.y ?
//                1.0d : (opmode.gamepad2.dpad_up ? 1.0d : 0.3d));
//
//        left_claw.setPosition(opmode.gamepad2.y ?
//                0 : (opmode.gamepad2.dpad_up ? 0.3d : 1.0d));

        opmode.telemetry.addData("Arm: ", armPower);
        opmode.telemetry.addData("Hand: ", handPosition);
        opmode.telemetry.addData("Claw: ", opmode.gamepad2.y);
        opmode.telemetry.addData("Right claw: ", opmode.gamepad2.dpad_up);
        opmode.telemetry.addData("Left claw: ", opmode.gamepad2.dpad_down);

        opmode.telemetry.addLine("Dev:");
        opmode.telemetry.addData("Average arm position: ", avgPosition);
    }

    public void init() {
        right_arm = opmode.hardwareMap.get(DcMotor.class, "right_arm");
        left_arm = opmode.hardwareMap.get(DcMotor.class, "left_arm");

        hand = opmode.hardwareMap.get(Servo.class, "hand");
        right_claw = opmode.hardwareMap.get(Servo.class, "right_claw");
        left_claw = opmode.hardwareMap.get(Servo.class, "left_claw");

        right_arm.setDirection(DcMotorSimple.Direction.FORWARD);
        left_arm.setDirection(DcMotorSimple.Direction.REVERSE);

        right_claw.setDirection(Servo.Direction.FORWARD);
        left_claw.setDirection(Servo.Direction.REVERSE);

        hand.setPosition(handPosition);
        right_claw.setPosition(_config.CLAW_OPEN);
        left_claw.setPosition(_config.CLAW_OPEN);
    }
}
