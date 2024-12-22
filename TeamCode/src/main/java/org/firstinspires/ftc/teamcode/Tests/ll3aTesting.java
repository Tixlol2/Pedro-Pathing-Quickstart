package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ll3aTesting extends LinearOpMode {

    public static int pipeline = 0;
    double angle;
    Limelight3A ll3a;
    String color;
    Servo claw;
    Servo rotate;

    double rotatePos;
    double rotateTarget;



    @Override
    public void runOpMode() throws InterruptedException {


        ll3a = hardwareMap.get(Limelight3A.class, "LL3A");
        claw = hardwareMap.get(Servo.class, "claw");
        rotate = hardwareMap.get(Servo.class, "rotate");


        while (opModeInInit()) {
           if(ll3a.isRunning()){
               telemetry.addLine("Should be good!");
               telemetry.update();
           } else {
               ll3a.pipelineSwitch(pipeline);
               ll3a.setPollRateHz(100);
               ll3a.start();
           }
        }

        while (opModeIsActive()) {

            //Main loop
            LLResult result = ll3a.getLatestResult();
            rotatePos = rotate.getPosition();
            if(!(Math.abs(rotatePos - rotateTarget) >= 0 &&  Math.abs(rotatePos - rotateTarget) <= .05)){
                rotate.setPosition(rotateTarget);
            }


            // Getting numbers from Python
            //TODO: Make this work with multiple blocks in the FOV, shouldn't be too hard in theory?
            //if(result.isValid()){
            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                if(pythonOutputs[0] == 0){telemetry.addLine("Either we are really good at our jobs \n or the SS isn't detecting for some reason");}
                else {
                    color = determineColor(pythonOutputs);
                    angle = pythonOutputs[0];
                    telemetry.addData("Color", color);
                    telemetry.addData("Angle in Degrees", angle);
                    telemetry.addData("Angle Target", rotateTarget);
                    telemetry.addData("Rotation Position", rotatePos);
                    telemetry.addData("Checking if >= 50", 90 - Math.abs(angle));
                if(90 - Math.abs(angle) >= 50){rotateTarget = .5;}
                else{rotateTarget = 1;}

            }
            } else {
                telemetry.addLine("No data received from Limelight.");
                }
            long staleness = result.getStaleness();
            if (staleness < 100) { // Less than 100 milliseconds old
                telemetry.addData("Data", "Good");
            } else {
                telemetry.addData("Data", "Old (" + staleness + " ms)");
            }
            //} else {telemetry.addLine("Result is not valid");}









            telemetry.update();
        }

    }

    public String determineColor(double[] outputs){
        int colorIndex = (int) outputs[1];

        if(colorIndex == 0){return "Red";}
        else if(colorIndex == 1){return "Blue";}
        else if(colorIndex == 2){return "Yellow";}

        return "Unknown";


    }



}




