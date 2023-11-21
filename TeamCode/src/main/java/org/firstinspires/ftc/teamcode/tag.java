package org.firstinspires.ftc.teamcode;

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

    public tag(LinearOpMode _opmode) {
        opmode = _opmode;
    }

    public void detection() {
        List<AprilTagDetection> detections = _tag.getDetections();
        opmode.telemetry.addData("Detections: ", detections);

        for (AprilTagDetection detection : detections) {
            if (detection == null) {
                opmode.telemetry.addData("?", "There is nothing we can do.");
                continue;
            }
            opmode.telemetry.addData("aaa", "there is something");
        }
    }

    public void init() {
        _tag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .build();

        _visionPortal = new VisionPortal
                .Builder()
                .setCamera(opmode.hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(_tag)
                .enableLiveView(true)
                .build();
    }
}
