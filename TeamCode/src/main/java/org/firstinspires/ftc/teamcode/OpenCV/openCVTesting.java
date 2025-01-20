package org.firstinspires.ftc.teamcode.OpenCV;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.google.blocks.ftcrobotcontroller.util.CurrentGame;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@TeleOp
@Config
public class openCVTesting extends LinearOpMode {


    Follower follower;
    ArrayList<AprilTagDetection> detections;
    public static int targetTagID = 0;
    int closestTagID;
    Pose target;
    sampleProcessor sampleProcessor = new sampleProcessor();
    VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        //init once

        telemetry = FtcDashboard.getInstance().getTelemetry();

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
        follower = new Follower(hardwareMap);

        while (!isStarted()) {
            //Init Loop
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addLine("Should only run the OpMode when this ^^^");
            telemetry.addLine("Says 'Streaming' or else the entire thing will break");
            telemetry.update();
        }

        while(isStarted() && !isStopRequested()){
            //Main loop
            double closestRange = 1000;
            detections = aprilTagProcessor.getDetections();


            telemetry.addData("Number of Tags ", detections.size());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Position of Sample", sampleProcessor.getPosition());
            telemetry.addLine();
            for (AprilTagDetection detection : detections) {
                if (closestRange > (Math.abs(detection.ftcPose.range))){
                    closestRange = Math.abs(detection.ftcPose.range);
                    closestTagID = detection.id;
                }
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
                    telemetry.addData("Range ", detection.ftcPose.range);
                    if(detection.id == closestTagID){
                        target = new Pose(detection.ftcPose.x, detection.ftcPose.y);
                        target.add(new Pose(0, 2));
                        follower.followPath(new Path(new BezierLine(new Point(follower.getPose()), new Point(target))));

                    }
                }
            }

            follower.updatePose();
            telemetry.update();



        }



    }

}
