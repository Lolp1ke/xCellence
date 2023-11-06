package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main", group = "xCellence")
public class main extends LinearOpMode {
    private final movement _movement = new movement(this);
    private final mechanism _mechanism = new mechanism(this);

    @Override()
    public void runOpMode() {
        this._movement.init();
        this._mechanism.init();

        telemetry.addData("Status", "Initialized successfully");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            new Thread(this._movement::run).start();
            new Thread(this._mechanism::run).start();
        }
    }
}
