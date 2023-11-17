package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class mechanism {
    private final LinearOpMode opMode;
    private final config _config = new config();

    private DcMotor right_arm;
    private DcMotor left_arm;

    private Servo hand;
    private Servo claw;

    private double handPosition = 0.0d;

    public mechanism(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void run() {
        boolean isBoosted = opMode.gamepad2.right_bumper;
        boolean isSlowed = opMode.gamepad2.left_bumper;

        double armPower = opMode.gamepad2.right_stick_y * (isBoosted ? _config.ARM_BOOST : (isSlowed ? _config.ARM_DECELERATION : _config.ARM_SPEED));

        if (opMode.gamepad2.x) {
            handPosition = 1.0d;
        } else if (opMode.gamepad2.a) {
            handPosition = 0.5d;
        } else if (opMode.gamepad2.b) {
            handPosition = 0.0d;
        }

        right_arm.setPower(armPower);
        left_arm.setPower(armPower);

        hand.setPosition(handPosition);
        claw.setPosition(opMode.gamepad2.y ? 0.8 : 0.6);

        opMode.telemetry.addData("Arm: ", armPower);

        opMode.telemetry.addData("Hand: ", handPosition);
    }

    public void init() {
        right_arm = opMode.hardwareMap.get(DcMotor.class, "right_arm");
        left_arm = opMode.hardwareMap.get(DcMotor.class, "left_arm");

        hand = opMode.hardwareMap.get(Servo.class, "hand");
        claw = opMode.hardwareMap.get(Servo.class, "claw");

        right_arm.setDirection(DcMotorSimple.Direction.FORWARD);
        left_arm.setDirection(DcMotorSimple.Direction.REVERSE);

        hand.setPosition(0);
        claw.setPosition(0);
    }
}
