package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Testing New")
public class AprilTag extends LinearOpMode {
    // April Tag Processor Stuff
    AprilTagProcessor myAprilTagProcessor;
    List<AprilTagDetection> myAprilTagDetections;  // list of all detections
    AprilTagLibrary library;
    AprilTagLibrary.Builder libraryBuilder;
    // Vision Portal Stuff
    VisionPortal portal;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;
    @Override
    public void runOpMode() throws InterruptedException {
        buildAprilTags(); buildVisionPortal();
        waitForStart();
        while(opModeIsActive()) {
            detectAprilTags();
            telemetry.update();
        }
    }
    public void detectAprilTags(){
        myAprilTagDetections = myAprilTagProcessor.getDetections();

        for (AprilTagDetection detection : myAprilTagDetections) {
            telemetry.addData("April Tag ID", detection.id);
            if (detection.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                AprilTagMetadata data = detection.metadata;

                telemetry.addData("April Tag Name", data.name);
                telemetry.addData("April Tag Distance", data.distanceUnit);
                telemetry.addData("April Tag Orientation", data.fieldOrientation);
                telemetry.addData("April Tag Position", data.fieldPosition);
                telemetry.addData("April Tag Size", data.tagsize);
            }
        }
    }

    public void buildAprilTags(){
        libraryBuilder = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .addTag(55, "Our Awesome Team Tag", 3.5, DistanceUnit.INCH);

        library = libraryBuilder.build();

        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();
    }
    public void buildVisionPortal() {
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .build();
    }
}
