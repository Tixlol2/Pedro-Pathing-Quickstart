package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;




@TeleOp(name="servoTest", group = "Drive")
@Disabled
public class servoTest extends LinearOpMode {

    //Class def


    Servo servo;
    double servoTarget;


    @Override
    public void runOpMode() throws InterruptedException {

        servo = hardwareMap.get(Servo.class, "Test");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();
        //During Initialization:


        while (opModeIsActive()) {

            if (gamepad2.b) {
                servoTarget = 0;
            } else if (gamepad2.a) {
                servoTarget = 1;
            }
            else if (gamepad2.x){
                servoTarget = 0.5;
            }


            if(servo.getPosition() != servoTarget){servo.setPosition(servoTarget);}

            telemetry.addData("Servo Target", servoTarget);
            telemetry.addData("Servo Position", servo.getPosition());
            telemetry.update();



        }
    }
}
