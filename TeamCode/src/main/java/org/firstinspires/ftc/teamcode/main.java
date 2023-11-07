package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "main", group = "!xCellence")
public class main extends LinearOpMode {
    private final movement2wd _movement2wd = new movement2wd(this);
    private final movement4wd _movement4wd = new movement4wd(this);
    private final mechanism _mechanism = new mechanism(this);

    private boolean is4wd = false;

    @Override()
    public void runOpMode() {
        this._movement4wd.init();
        this._movement2wd.init();
        this._mechanism.init();

        while (opModeInInit()) {
            if (gamepad1.a && gamepad1.dpad_up) this.is4wd = true;
            else if (gamepad1.a && gamepad1.dpad_down) this.is4wd = false;

            telemetry.addData("Drive type: ", this.is4wd ? "4wd" : "2wd");
            telemetry.update();
        }

        waitForStart();
        while (opModeIsActive()) {
            if (is4wd)
                new Thread(this._movement4wd::run).start();
            else
                new Thread(this._movement2wd::run).start();

            new Thread(this._mechanism::run).start();
        }
    }
}
