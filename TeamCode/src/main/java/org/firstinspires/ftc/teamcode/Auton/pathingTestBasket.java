package org.firstinspires.ftc.teamcode.Auton;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Intake.ClawSubsystem;
import org.firstinspires.ftc.teamcode.Stage1.ArmSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous
public class pathingTestBasket extends OpMode {


    Follower follower;
    private Timer opmodeTimer, pathTimer;
    private int pathState;
    ArmSubsystem armSubsystem;
    ClawSubsystem clawSubsystem;
    CommandScheduler commandScheduler;
    Path tempPath;

    PathChain startToBasket, pickupSample1, returnToBasket1, pickupSample2, returnToBasket2, pickupSample3, returnToBasket3, park;
    autonPosesPedro autonPoses = new autonPosesPedro();

    public void buildPaths() {

        park = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoses.basketScore), /* CONTROL POINT ->>> */ new Point(autonPoses.basketParkCP1), new Point(autonPoses.basketPark))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.basketPark.getHeading())
                .build();

        startToBasket = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.startPoseBasket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.startPoseBasket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

        pickupSample1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.basketScore), new Point(autonPoses.samplePickup1Basket))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.samplePickup1Basket.getHeading())
                .build();

        returnToBasket1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup1Basket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.samplePickup1Basket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

        pickupSample2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.basketScore), new Point(autonPoses.samplePickup2Basket))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.samplePickup2Basket.getHeading())
                .build();

        returnToBasket2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup2Basket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.samplePickup2Basket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

        pickupSample3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.basketScore), new Point(autonPoses.samplePickup3Basket))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.samplePickup3Basket.getHeading())
                .build();

        returnToBasket3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup3Basket), new Point(autonPoses.basketScore))))
                .setLinearHeadingInterpolation(autonPoses.samplePickup3Basket.getHeading(), autonPoses.basketScore.getHeading())
                .build();

    }

    public void autonomousPathUpdate(){

        switch(pathState){

            case 0:
                follower.followPath(startToBasket, true);
                setPathState(1);
            break;
            case 1:
                //Getting to scoring pos
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the basketScore position */
                if(     follower.getPose().getX() - 1 < autonPoses.basketScore.getX() && follower.getPose().getX() + 1 > autonPoses.basketScore.getX() &&
                        follower.getPose().getY() - 1 < autonPoses.basketScore.getY() && follower.getPose().getY() + 1 > autonPoses.basketScore.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(5) < autonPoses.basketScore.getHeading() && follower.getPose().getHeading() + Math.toRadians(5) > autonPoses.basketScore.getHeading()) {
                    //follower.followPath(pickupSample1, true);
                    ClawSubsystem.setWristPosition(0.5);
                    setPathState(14);
                }
            break;
            case 14:
                if (pathTimer.getElapsedTime()> 3500) {
                    follower.followPath(pickupSample1,true);
                    setPathState(2);
                    ClawSubsystem.open();
                    ClawSubsystem.setWristPosition(0);
                    ArmSubsystem.setPos(10,60);
                }else if (pathTimer.getElapsedTime() > 2500) {
                    ClawSubsystem.open();
                } else if(pathTimer.getElapsedTime() > 2000) {
                    ClawSubsystem.setWristPosition(1);
                    ArmSubsystem.setPos(50,107);
                }else if(pathTimer.getElapsedTime() > 500) {
                    ArmSubsystem.setPos(50,100);
                }

                break;
            case 2:
                //picking up first sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup1Basket heading */
                if(     follower.getPose().getX() - 1 < autonPoses.samplePickup1Basket.getX() && follower.getPose().getX() + 1 > autonPoses.samplePickup1Basket.getX() &&
                        follower.getPose().getY() - 1 < autonPoses.samplePickup1Basket.getY() && follower.getPose().getY() + 1 > autonPoses.samplePickup1Basket.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(5) < autonPoses.samplePickup1Basket.getHeading() && follower.getPose().getHeading() + Math.toRadians(5) > autonPoses.samplePickup1Basket.getHeading()) {
                    setPathState(8);
                }
            break;
            case 8:
                if (pathTimer.getElapsedTime() > 2500) {
                    ArmSubsystem.setPos(0,45);
                    follower.followPath(returnToBasket1, true);
                    setPathState(3);
                }else if (pathTimer.getElapsedTime() > 1500) {
                    ClawSubsystem.close();
                } else if (pathTimer.getElapsedTime() > 1000) {
                    ArmSubsystem.setPos(new Vector2d(35, 3));
                } else {
                    ClawSubsystem.setWristPosition(0);
                    ClawSubsystem.open();
                    ArmSubsystem.setPos(new Vector2d(35, 12));
                }
            break;
            case 3:
                //scoring first sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                if(follower.getPose().getX() - 1.5 < autonPoses.basketScore.getX() && follower.getPose().getX() + 1.5 > autonPoses.basketScore.getX() &&
                        follower.getPose().getY() - 1.5 < autonPoses.basketScore.getY() && follower.getPose().getY() + 1.5 > autonPoses.basketScore.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(5) < autonPoses.basketScore.getHeading() && follower.getPose().getHeading() + Math.toRadians(5) > autonPoses.basketScore.getHeading()) {
                    setPathState(9);
                }
            break;
            case 9:
                if (pathTimer.getElapsedTime()> 3500) {
                    follower.followPath(pickupSample2,true);
                    setPathState(4);
                    ClawSubsystem.open();
                    ClawSubsystem.setWristPosition(0);
                    ArmSubsystem.setPos(10,60);
                }else if (pathTimer.getElapsedTime() > 2800) {
                    ClawSubsystem.open();
                } else if(pathTimer.getElapsedTime() > 2000) {
                    ClawSubsystem.setWristPosition(1);
                    ArmSubsystem.setPos(50,109);
                }else if(pathTimer.getElapsedTime() > 500) {
                    ArmSubsystem.setPos(50,100);
                }

            break;
            case 4:
                //picking up second sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup2Basket heading */
                if(follower.getPose().getX() - 1 < autonPoses.samplePickup2Basket.getX() && follower.getPose().getX() + 1 > autonPoses.samplePickup2Basket.getX() &&
                        follower.getPose().getY() - 1 < autonPoses.samplePickup2Basket.getY() && follower.getPose().getY() + 1 > autonPoses.samplePickup2Basket.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(5) < autonPoses.samplePickup2Basket.getHeading() && follower.getPose().getHeading() + Math.toRadians(5) > autonPoses.samplePickup2Basket.getHeading()) {
                    setPathState(10);
                }
            break;
            case 10:
                if (pathTimer.getElapsedTime() > 2500) {
                    ArmSubsystem.setPos(0,45);
                    follower.followPath(returnToBasket1, true);
                    setPathState(5);
                }else if (pathTimer.getElapsedTime() > 1500) {
                    ClawSubsystem.close();
                } else if (pathTimer.getElapsedTime() > 1000) {
                    ArmSubsystem.setPos(new Vector2d(36, 3));
                } else {
                    ClawSubsystem.setWristPosition(0);
                    ClawSubsystem.open();
                    ArmSubsystem.setPos(new Vector2d(36, 12));
                }


            break;
            case 5:
                //scoring second sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                if(follower.getPose().getX() - 1 < autonPoses.basketScore.getX() && follower.getPose().getX() + 1 > autonPoses.basketScore.getX() &&
                        follower.getPose().getY() - 1 < autonPoses.basketScore.getY() && follower.getPose().getY() + 1 > autonPoses.basketScore.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(5) < autonPoses.basketScore.getHeading() && follower.getPose().getHeading() + Math.toRadians(5) > autonPoses.basketScore.getHeading()) {
                    setPathState(11);
                }
            break;
            case 11:
                if (pathTimer.getElapsedTime()> 3700) {
                    follower.followPath(pickupSample3,true);
                    setPathState(6);
                    ClawSubsystem.open();
                    ClawSubsystem.setWristPosition(0);
                    ArmSubsystem.setPos(10,60);
                }else if (pathTimer.getElapsedTime() > 2500) {
                    ClawSubsystem.open();
                } else if(pathTimer.getElapsedTime() > 2000) {
                    ClawSubsystem.setWristPosition(1);
                    ArmSubsystem.setPos(50,105);
                }else if(pathTimer.getElapsedTime() > 500) {
                    ArmSubsystem.setPos(50,90);
                }

                break;
            case 6:
                //picking up third sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup3Basket heading */
                if(follower.getPose().getX() - 1 < autonPoses.samplePickup3Basket.getX() && follower.getPose().getX() + 1 > autonPoses.samplePickup3Basket.getX() &&
                        follower.getPose().getY() - 1 < autonPoses.samplePickup3Basket.getY() && follower.getPose().getY() + 1 > autonPoses.samplePickup3Basket.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(5) < autonPoses.samplePickup3Basket.getHeading() && follower.getPose().getHeading() + Math.toRadians(5) > autonPoses.samplePickup3Basket.getHeading()) {
                    setPathState(12);
                }
            break;
            case 12:
                if (pathTimer.getElapsedTime() > 2500) {
                    ArmSubsystem.setPos(0,60);
                    follower.followPath(returnToBasket1, true);
                    setPathState(7);
                }else if (pathTimer.getElapsedTime() > 1500) {
                    ClawSubsystem.close();
                } else if (pathTimer.getElapsedTime() > 1000) {
                    ArmSubsystem.setPos(new Vector2d(35, 3));
                } else {
                    ClawSubsystem.setWristPosition(0);
                    ClawSubsystem.open();
                    ArmSubsystem.setPos(new Vector2d(35, 12));
                }
            break;
            case 7:
                // scoring third sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                if(follower.getPose().getX() - 1 < autonPoses.basketScore.getX() && follower.getPose().getX() + 1 > autonPoses.basketScore.getX() &&
                        follower.getPose().getY() - 1 < autonPoses.basketScore.getY() && follower.getPose().getY() + 1 > autonPoses.basketScore.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(5) < autonPoses.basketScore.getHeading() && follower.getPose().getHeading() + Math.toRadians(5) > autonPoses.basketScore.getHeading()) {
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTime()> 3500) {
                    follower.followPath(park,true);
                    ClawSubsystem.open();
                    ClawSubsystem.setWristPosition(0);
                    ArmSubsystem.setPos(10,60);
                }else if (pathTimer.getElapsedTime() > 2500) {
                    ClawSubsystem.open();
                } else if(pathTimer.getElapsedTime() > 2000) {
                    ClawSubsystem.setWristPosition(1);
                    ArmSubsystem.setPos(50,103);
                }else if(pathTimer.getElapsedTime() > 500) {
                    ArmSubsystem.setPos(50,90);
                }

                break;
        }

    }

    public void setPathState(int state){

        pathState = state;
        pathTimer.resetTimer();
    }




    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        armSubsystem = new ArmSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);
        commandScheduler = CommandScheduler.getInstance();

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(autonPoses.startPoseBasket);
        follower.getPose().setHeading(autonPoses.startPoseBasket.getHeading());
        ClawSubsystem.setAnglePosition(1);
        ArmSubsystem.setPos(2,30);


        buildPaths();
    }

    @Override
    public void init_loop(){
        //Looped
        ClawSubsystem.close();
        ArmSubsystem.update();
        commandScheduler.run();

    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();
        ArmSubsystem.update();
        commandScheduler.run();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Arm Target Ext: ", ArmSubsystem.getExtTarget());
        telemetry.addData("Test: ", Math.toDegrees(follower.getPose().getHeading() - Math.toRadians(3)));
        telemetry.update();
    }


    @Override
    public void start() {
        //Called once at the start of run, to setup basic stuff
        opmodeTimer.resetTimer();
        setPathState(0);

    }

    public boolean toleranceCheck(double toCheck, double tolerance, double min, double max){

        return toCheck - tolerance > min && toCheck + tolerance < max;


    }


}
