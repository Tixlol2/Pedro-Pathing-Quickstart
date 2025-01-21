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
                if(     follower.getPose().getX() - 1 < autonPoses.samplePickup1Basket.getX() && follower.getPose().getX() + 1 > autonPoses.samplePickup1Basket.getX() &&
                        follower.getPose().getY() - 1 < autonPoses.samplePickup1Basket.getX() && follower.getPose().getY() + 1 > autonPoses.samplePickup1Basket.getY() &&
                        follower.getPose().getHeading() - Math.toRadians(10) < autonPoses.samplePickup1Basket.getHeading() && follower.getPose().getHeading() + Math.toRadians(10) > autonPoses.samplePickup1Basket.getHeading()) {
                    follower.followPath(pickupSample1, true);
                    setPathState(2);
                }

                break;
            case 2:
                //picking up first sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup1Basket heading */
                if(follower.getPose().getHeading() - Math.toRadians(10) < autonPoses.samplePickup1Basket.getHeading() && follower.getPose().getHeading() + Math.toRadians(10) > autonPoses.samplePickup1Basket.getHeading()) {
                    ClawSubsystem.setWristPosition(0);
                    ArmSubsystem.setPos(new Vector2d(32.5,5.2));
                    if (pathTimer.getElapsedTime() > 700) {
                        ClawSubsystem.close();
                    }

                    if (pathTimer.getElapsedTime() > 1000) {
                        ArmSubsystem.setPos(new Vector2d(18,18));

                        follower.followPath(returnToBasket1, true);
                        setPathState(3);
                    }

                }
                break;
            case 3:
                //scoring first sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                
                if(follower.getPose().getHeading() - Math.toRadians(3) < autonPoses.basketScore.getHeading() && follower.getPose().getHeading() + Math.toRadians(3) > autonPoses.basketScore.getHeading()) {

                    if (pathTimer.getElapsedTime() > 1000)
                        follower.followPath(pickupSample2,true);
                        setPathState(4);
                }
                break;
            case 4:
                //picking up second sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup2Basket heading */
                if(follower.getPose().getHeading() - Math.toRadians(3) < autonPoses.samplePickup2Basket.getHeading() && follower.getPose().getHeading() + Math.toRadians(3) > autonPoses.samplePickup2Basket.getHeading()) {
                    follower.followPath(returnToBasket2,true);
                    if (pathTimer.getElapsedTime() > 1000)
                        setPathState(5);
                }
                break;
            case 5:
                //scoring second sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                if(follower.getPose().getHeading() - Math.toRadians(3) < autonPoses.basketScore.getHeading() && follower.getPose().getHeading() + Math.toRadians(3) > autonPoses.basketScore.getHeading()) {
                    follower.followPath(pickupSample3,true);
                    if (pathTimer.getElapsedTime() > 1000)
                        setPathState(6);
                }
                break;
            case 6:
                //picking up third sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the samplePickup3Basket heading */
                if(follower.getPose().getHeading() - Math.toRadians(3) < autonPoses.samplePickup3Basket.getHeading() && follower.getPose().getHeading() + Math.toRadians(3) > autonPoses.samplePickup3Basket.getHeading()) {
                    follower.followPath(returnToBasket3,true);
                    if (pathTimer.getElapsedTime() > 1000)
                        setPathState(7);
                }
                break;
            case 7:
                // scoring third sample
                /* This case checks the robot's heading and will wait until the robot heading is close (within 3 degrees) from the basketScore heading */
                if(follower.getPose().getHeading() - Math.toRadians(3) < autonPoses.basketScore.getHeading() && follower.getPose().getHeading() + Math.toRadians(3) > autonPoses.basketScore.getHeading()) {
                    follower.followPath(park,true);
                    if (pathTimer.getElapsedTime() > 1000)
                        setPathState(-1);
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
        ArmSubsystem.setPos(0, 0);


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
        follower.telemetryDebug(telemetry);
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
