package org.firstinspires.ftc.teamcode.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "main", group = "!xCellence")
public class main extends LinearOpMode {
    private final movement2wd _movement2wd = new movement2wd(this);
    private final mechanism _mechanism = new mechanism(this);

    @Override
    public void runOpMode() {
        _mechanism.init();
        _movement2wd.init();

        telemetry.addData("Status: ", "vroom vroom");
        telemetry.update();
        
        waitForStart();
        while (opModeIsActive()) {
            _mechanism.run();
            _movement2wd.car();

            telemetry.update();
        }
    }
}