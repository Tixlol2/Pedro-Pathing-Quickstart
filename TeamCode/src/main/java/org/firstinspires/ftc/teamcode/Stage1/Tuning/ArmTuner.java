package org.firstinspires.ftc.teamcode.Stage1.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Stage1.ArmSubsystem;
@Disabled
@Config
@TeleOp
public class ArmTuner extends LinearOpMode {




    public static double x = 18;
    public static double y = 0;



    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());




        waitForStart();


        ArmSubsystem armSubsystem = new ArmSubsystem(hardwareMap, "armExtendUp", "armExtendDown", "armAngleLeft", "armAngleRight");



        while(!isStopRequested()){








            telemetry.addData("Angle Ticks", ArmSubsystem.getAnglePos());
            telemetry.addData("Angle Degrees", ArmSubsystem.getAnglePosDEG());
            telemetry.addData("Extension Ticks", ArmSubsystem.getExtenderPos());
            telemetry.addData("Extension Inches", ArmSubsystem.getExtenderPosIN());


            telemetry.addData("Current REAL Angle Target", armSubsystem.getAngleTargetDG());
            telemetry.addData("Current REAL Extend Target", armSubsystem.getExtTargetIN());

            telemetry.addData("Current Set X: ",x );
            telemetry.addData("Current Set Y: ",y );

//            position = armSubsystem.getPosition();

            telemetry.addData("Current X: ", );
            telemetry.addData("Current y: ", );

            telemetry.update();



        }



    }
}
