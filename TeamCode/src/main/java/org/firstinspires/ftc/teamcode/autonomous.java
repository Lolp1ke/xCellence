package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autonomous", group = "!xCellence")
public class autonomous extends LinearOpMode {
    private final tag _tag = new tag(this);

    @Override
    public void runOpMode() {
        _tag.init();

        waitForStart();
        while (opModeIsActive()) {
            _tag.detection();

            telemetry.update();
        }
    }
}
