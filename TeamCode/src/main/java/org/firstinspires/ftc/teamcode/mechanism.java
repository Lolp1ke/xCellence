package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class mechanism {
    private final LinearOpMode opMode;
    private final config _config = new config();

    private DcMotor rightArm;
    private DcMotor leftArm;

    private Servo rightHand;
    private Servo leftHand;


    private double handPosition = 0d;

    public mechanism(LinearOpMode _opMode) {
        opMode = _opMode;
    }

    public void init() {
        rightArm = opMode.hardwareMap.get(DcMotor.class, "right_arm");
        leftArm = opMode.hardwareMap.get(DcMotor.class, "left_arm");

        rightHand = opMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand = opMode.hardwareMap.get(Servo.class, "left_hand");

        rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        rightHand.setPosition(handPosition);
        leftHand.setPosition(handPosition);
    }

    public void run() {
        boolean isBoosted = opMode.gamepad2.right_bumper;

        double rightArmPower = -opMode.gamepad2.right_stick_y * (isBoosted ? _config.ARM_BOOST : _config.ARM_SPEED);
        double leftArmPower = -opMode.gamepad2.left_stick_y * (isBoosted ? _config.ARM_BOOST : _config.ARM_SPEED);

        if (opMode.gamepad2.x) {
            handPosition = 1.0d;
        } else if (opMode.gamepad2.a) {
            handPosition = 0.5d;
        } else if (opMode.gamepad2.b) {
            handPosition = 0.0d;
        }

        rightArm.setPower(rightArmPower);
        leftArm.setPower(leftArmPower);

        rightHand.setPosition(handPosition);
        leftHand.setPosition(handPosition);

        opMode.telemetry.addData("Arm right/left: ", "%.2/%.2", rightArmPower, leftArmPower);
        opMode.telemetry.addData("Hand position: ", handPosition);
        opMode.telemetry.update();
    }
}
