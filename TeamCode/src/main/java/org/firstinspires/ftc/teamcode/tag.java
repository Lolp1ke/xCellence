package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class tag {
    private final LinearOpMode opmode;

    private VisionPortal _visionPortal;
    private AprilTagProcessor _tag;
    private int napoleonSoundID;
    private boolean napoleonFound = false;
    private boolean napoleonLaunched = false;

    public tag(LinearOpMode _opmode) {
        opmode = _opmode;
    }


    public void detection() {
        List<AprilTagDetection> detections = _tag.getDetections();
        opmode.telemetry.addData("Detections: ", detections);
        opmode.telemetry.addData("on?: ", SoundPlayer.getInstance().isLocalSoundOn());
        opmode.telemetry.addData("There is nothing we can do.\n", "Dans mon esprit tout divague\n" +
                "Je me perds dans tes yeux\n" +
                "Je me noie dans la vague\n" +
                "De ton regard amoureux\n" +
                "Je ne veux que ton âme\n" +
                "Divaguant sur ma peau\n" +
                "Une fleur, une femme\n" +
                "Dans ton cœur Roméo\n" +
                "Je ne suis que ton ombre");

        if (napoleonFound && !napoleonLaunched && detections.size() == 0) {
            SoundPlayer.getInstance().startPlaying(opmode.hardwareMap.appContext, napoleonSoundID);
            napoleonLaunched = true;
        }

        for (AprilTagDetection detection : detections) {
            if (detections.size() > 0) {
                SoundPlayer.getInstance().stopPlayingAll();
                opmode.telemetry.addData("there is something i can do", detection.id);
            }
        }

        opmode.telemetry.update();
    }

    public void init() {
        napoleonSoundID = opmode.hardwareMap.appContext.getResources().getIdentifier("napoleon", "raw", opmode.hardwareMap.appContext.getPackageName());

        if (napoleonSoundID != 0)
            napoleonFound = SoundPlayer.getInstance().preload(opmode.hardwareMap.appContext, napoleonSoundID);


        _tag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        _visionPortal = new VisionPortal
                .Builder()
                .setCamera(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(_tag)
                .enableLiveView(true)
                .build();
    }
}
