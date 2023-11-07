package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main", group = "xCellence")
public class main extends LinearOpMode {
    private final movement4wd _movement4wd = new movement4wd(this);
    private final movement2wd _movement2wd = new movement2wd(this);
    private final mechanism _mechanism = new mechanism(this);


    @Override()
    public void runOpMode() {
        this._movement4wd.init();
        this._mechanism.init();

        telemetry.addData("Status", "Initialized successfully");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            new Thread(this._movement4wd::run).start();
            new Thread(this._mechanism::run).start();
        }
    }
}
