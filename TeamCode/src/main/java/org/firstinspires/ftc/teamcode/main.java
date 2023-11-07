package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "main", group = "!xCellence")
public class main extends LinearOpMode {
    @Override()
    public void runOpMode() {
        telemetry.addData("Status: ", "Success");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
        }
    }
}
