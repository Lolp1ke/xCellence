package org.firstinspires.ftc.teamcode.autonomous;

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


    public List<AprilTagDetection> detection() {
        List<AprilTagDetection> detections = _tag.getDetections();
        opmode.telemetry.addData("Tag detections: ", detections);

        return detections;
//        opmode.telemetry.addLine("There is nothing we can do.\n");
//        opmode.telemetry.addLine("Dans mon esprit tout divague\n" +
//                "Je me perds dans tes yeux\n" +
//                "Je me noie dans la vague\n" +
//                "De ton regard amoureux\n" +
//                "Je ne veux que ton âme\n" +
//                "Divaguant sur ma peau\n" +
//                "Une fleur, une femme\n" +
//                "Dans ton cœur Roméo\n" +
//                "Je ne suis que ton ombre");
    }

    public void init() {
        _tag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        _visionPortal = new VisionPortal
                .Builder()
                .setCamera(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(_tag)
                .enableLiveView(true)
                .build();
    }
}
