package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled()
public class mechanism {
    private final LinearOpMode opMode;

    private DcMotor rightArm = null;
    private DcMotor leftArm = null;

    private Servo rightHand = null;
    private Servo leftHand = null;

    private final double ARM_POWER = 0.7d;
    private final double SERVO_RESET = 0;

    public mechanism(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        this.rightArm = this.opMode.hardwareMap.get(DcMotor.class, "right_arm");
        this.leftArm = this.opMode.hardwareMap.get(DcMotor.class, "left_arm");

        this.rightHand = this.opMode.hardwareMap.get(Servo.class, "right_hand");
        this.leftHand = this.opMode.hardwareMap.get(Servo.class, "right_hand");

        this.rightArm.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftArm.setDirection(DcMotorSimple.Direction.FORWARD);

        this.rightHand.setPosition(this.SERVO_RESET);
        this.leftHand.setPosition(this.SERVO_RESET);
    }

    public void run() {
    }
}
