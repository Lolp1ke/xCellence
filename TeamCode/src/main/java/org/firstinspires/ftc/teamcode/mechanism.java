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

    public mechanism(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        this.rightArm = this.opMode.hardwareMap.get(DcMotor.class, "right_arm");
        this.leftArm = this.opMode.hardwareMap.get(DcMotor.class, "left_arm");

        this.rightHand = this.opMode.hardwareMap.get(Servo.class, "right_hand");
        this.leftHand = this.opMode.hardwareMap.get(Servo.class, "left_hand");

        this.rightArm.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftArm.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightHand.setPosition(this.handPosition);
        this.leftHand.setPosition(this.handPosition);
    }

    public void run() {
        boolean isBoosted = this.opMode.gamepad2.right_bumper;

        double rightArmPower = -this.opMode.gamepad2.right_stick_y * (isBoosted ? this._config.ARM_BOOST : this._config.ARM_SPEED);
        double leftArmPower = -this.opMode.gamepad2.left_stick_y * (isBoosted ? this._config.ARM_BOOST : this._config.ARM_SPEED);

        if (this.opMode.gamepad2.x) {
            this.handPosition = 1.0d;
        } else if (this.opMode.gamepad2.a) {
            this.handPosition = 0.5d;
        } else if (this.opMode.gamepad2.b) {
            this.handPosition = 0.0d;
        }

        this.rightArm.setPower(rightArmPower);
        this.leftArm.setPower(leftArmPower);

        this.rightHand.setPosition(handPosition);
        this.leftHand.setPosition(handPosition);

        this.opMode.telemetry.addData("Arm right/left: ", "%.2/%.2", rightArmPower, leftArmPower);
        this.opMode.telemetry.addData("Hand position: ", "/", this.handPosition);
        this.opMode.telemetry.update();
    }
}
