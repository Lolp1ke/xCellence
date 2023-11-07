package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class movement4wd {
    private final LinearOpMode opMode;

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor leftRear;

    public movement4wd(LinearOpMode opMode) {
        this.opMode = opMode;
    }


    public void run() {
        double normalizer;

        double x = this.opMode.gamepad1.left_stick_x;
        double y = -this.opMode.gamepad1.left_stick_y;
        double angle = this.opMode.gamepad1.right_stick_x;

        double rightFrontPower = -x + y - angle;
        double leftFrontPower = x + y + angle;
        double rightRearPower = x + y - angle;
        double leftRearPower = -x + y + angle;

        normalizer = Math.max(
                Math.max(Math.abs(rightFrontPower), Math.abs(leftFrontPower)),
                Math.max(Math.abs(rightRearPower), Math.abs(leftRearPower))
        );

        if (normalizer > 1) {
            rightFrontPower /= normalizer;
            leftFrontPower /= normalizer;
            rightRearPower /= normalizer;
            leftRearPower /= normalizer;
        }

        this.rightFront.setPower(rightFrontPower);
        this.leftFront.setPower(leftFrontPower);
        this.rightRear.setPower(rightRearPower);
        this.leftRear.setPower(leftRearPower);

        this.opMode.telemetry.addData("Front right/left: ", "%.2/%.2", rightFrontPower, leftFrontPower);
        this.opMode.telemetry.addData("Rear right/left: ", "%.2/%.2", rightRearPower, leftRearPower);
        this.opMode.telemetry.update();
    }

    public void init() {
        this.rightFront = this.opMode.hardwareMap.get(DcMotor.class, "right_front");
        this.leftFront = this.opMode.hardwareMap.get(DcMotor.class, "right_front");
        this.rightRear = this.opMode.hardwareMap.get(DcMotor.class, "right_front");
        this.leftRear = this.opMode.hardwareMap.get(DcMotor.class, "right_front");

        this.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
