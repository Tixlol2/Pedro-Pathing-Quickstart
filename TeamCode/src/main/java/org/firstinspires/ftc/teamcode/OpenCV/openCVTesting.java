package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.google.blocks.ftcrobotcontroller.util.CurrentGame;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
public class openCVTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        //init once
        ArrayList<AprilTagDetection> detections;
        telemetry = FtcDashboard.getInstance().getTelemetry();

        sampleProcessor sampleProcessor = new sampleProcessor();

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                //What tags to look for
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                //DrawTagID on image
                .setDrawTagID(true)
                //Draw outline
                .setDrawTagOutline(true)
                //Draw axes
                .setDrawAxes(true)
                //Draw cube
                .setDrawCubeProjection(true)
                //Finish
                .build();
        VisionPortal visionPortal;
        visionPortal = new VisionPortal.Builder()
                //Get camera from hMap
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                //Add all needed processors (hand written)
                .addProcessor(aprilTagProcessor)
                .addProcessor(sampleProcessor)
                //Set camera resolution
                .setCameraResolution(new Size(640, 480))
                //No idea
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //Enable streaming to Dhub and FTC Dashboard (?)
                .enableLiveView(true)
                //If all proccesors disabled, stop streaming
                .setAutoStopLiveView(true)
                //finish
                .build();

        while (!isStarted()) {
            //Init Loop


            detections = aprilTagProcessor.getDetections();

            telemetry.addData("Number of Tags ", detections.size());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Position of Sample", sampleProcessor.getPosition());
            telemetry.addLine();
            for (AprilTagDetection detection : detections) {
                if(detection.metadata != null) {
                    telemetry.addData("Tag ID ", detection.id);
                    telemetry.addData("X Offset ", detection.ftcPose.x);
                    telemetry.addData("Y Offset ", detection.ftcPose.y);
                    telemetry.addData("Z Offset ", detection.ftcPose.z);
                    telemetry.addLine();
                    telemetry.addData("Pitch Offset ", detection.ftcPose.pitch);
                    telemetry.addData("Roll Offset ", detection.ftcPose.roll);
                    telemetry.addData("Yaw Offset ", detection.ftcPose.yaw);
                    telemetry.addLine();
                }
            }

            telemetry.update();



        }

        while(isStarted() && !isStopRequested()){
            //Main loop




        }



    }
}
