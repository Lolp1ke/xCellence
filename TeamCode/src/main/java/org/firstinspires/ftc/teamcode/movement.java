package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
public class movement {
    private final LinearOpMode opMode;

    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightRear = null;
    private DcMotor leftRear = null;

    private final double SPEED = 0.6d;
    private final double BOOST = 0.8d;

    private final double kP = 0.1d;
    private final double kI = 0.01d;
    private final double kD = 0.001d;

    private double targetPosition = 0;
    private double integral = 0;
    private double prevError = 0;

    public movement(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void runWithPID() {
        Gamepad gamepad = this.opMode.gamepad1;
        Telemetry telemetry = this.opMode.telemetry;

        double drivePower = -gamepad.left_stick_y;
        double turnPower = gamepad.left_stick_x;

        double currentAvgPosition = (this.rightFront.getCurrentPosition() + this.leftFront.getCurrentPosition() +
                this.rightRear.getCurrentPosition() + this.leftRear.getCurrentPosition()) / 4.0;

        double error = this.targetPosition - currentAvgPosition;

        double proportional = this.kP * error;
        this.integral += this.kI * error;
        double derivative = this.kD * (error - this.prevError);

        double output = proportional + this.integral + derivative;

        double rightFrontPower = drivePower - turnPower - output;
        double leftFrontPower = drivePower + turnPower + output;
        double rightRearPower = drivePower - turnPower + output;
        double leftRearPower = drivePower + turnPower - output;

        leftFrontPower = Range.clip(leftFrontPower, -1, 1);
        rightFrontPower = Range.clip(rightFrontPower, -1, 1);
        leftRearPower = Range.clip(leftRearPower, -1, 1);
        rightRearPower = Range.clip(rightRearPower, -1, 1);

        this.rightFront.setPower(rightFrontPower);
        this.leftFront.setPower(leftFrontPower);
        this.rightRear.setPower(rightRearPower);
        this.leftRear.setPower(leftRearPower);

        this.prevError = error;

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftRearPower, rightRearPower);
        telemetry.update();
    }

    public void run() {
        Gamepad gamepad = this.opMode.gamepad1;
        Telemetry telemetry = this.opMode.telemetry;
        double max;

        double axial = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double yaw = gamepad.right_stick_x;

        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftRearPower = axial - lateral + yaw;
        double rightRearPower = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftRearPower));
        max = Math.max(max, Math.abs(rightRearPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftRearPower /= max;
            rightRearPower /= max;
        }

        boolean isBoosted = gamepad.right_bumper;
        this.rightFront.setPower(rightFrontPower * (isBoosted ? BOOST : SPEED));
        this.leftFront.setPower(leftFrontPower * (isBoosted ? BOOST : SPEED));
        this.rightRear.setPower(rightRearPower * (isBoosted ? BOOST : SPEED));
        this.leftRear.setPower(leftRearPower * (isBoosted ? BOOST : SPEED));

        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftRearPower, rightRearPower);
        telemetry.update();
    }

    public void init() {
        this.rightFront = opMode.hardwareMap.get(DcMotor.class, "right_front");
        this.leftFront = opMode.hardwareMap.get(DcMotor.class, "left_front");
        this.rightRear = opMode.hardwareMap.get(DcMotor.class, "right_rear");
        this.leftRear = opMode.hardwareMap.get(DcMotor.class, "right_front");

        this.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
