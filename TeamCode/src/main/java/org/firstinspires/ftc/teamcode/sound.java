package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "sound")
public class sound extends LinearOpMode { // there is nothing we can do without speakers
    @Override
    public void runOpMode() {

        waitForStart();
        while (opModeIsActive()) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, R.raw.napoleon);
            while (SoundPlayer.getInstance().isLocalSoundOn()) ;
        }
    }
}
