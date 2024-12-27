package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Intake.Commands.clawCloseCommand;
import org.firstinspires.ftc.teamcode.Intake.clawSubsystem;
import org.firstinspires.ftc.teamcode.Stage1.armSubsystemPedro;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

@Autonomous
public class basketSide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

    telemetry = FtcDashboard.getInstance().getTelemetry();
    armSubsystemPedro armSubsystem = new armSubsystemPedro(hardwareMap);
    clawSubsystem clawSubsystem = new clawSubsystem(hardwareMap);
    Follower follower = new Follower(hardwareMap);
    follower.setStartingPose(new Pose(0, 0));
    clawCloseCommand clawClose = new clawCloseCommand(clawSubsystem);

    clawClose.schedule();
    CommandScheduler.getInstance().run();
    follower.update();
    waitForStart();



    while(isStarted() && !isStopRequested()){


    }









    }
}
