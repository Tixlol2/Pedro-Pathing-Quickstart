package org.firstinspires.ftc.teamcode.Auton;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
@Autonomous
public class pathingTestChamber extends OpMode {


    Follower follower;
    private Timer opmodeTimer, pathTimer;
    private int pathState;

    PathChain startToChamber, pickupSample1, goToObsZone1, pickupSample2, goToObsZone2, pickupSample3, goToObsZone3, park, sample3ToPickup, pickupToScore, scoreToPickup;
    autonPosesPedro autonPoses;

    public void buildPaths() {

        park = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoses.basketScore), /* CONTROL POINT ->>> */ new Point(autonPoses.basketParkCP1), new Point(autonPoses.basketPark))))
                .setLinearHeadingInterpolation(autonPoses.basketScore.getHeading(), autonPoses.basketPark.getHeading())
                .build();

        startToChamber = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.startPoseChamber), new Point(autonPoses.chamberScore))))
                .setConstantHeadingInterpolation(0)
                .build();

        pickupSample1 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoses.chamberScore),/* CONTROL POINT ->>> */ new Point(autonPoses.samplePickup1CP1), new Point(autonPoses.samplePickup1Chamber))))
                .setConstantHeadingInterpolation(0)
                .build();

        goToObsZone1 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup1Chamber), new Point(autonPoses.sampleReturn1Chamber))))
                .setConstantHeadingInterpolation(0)
                .build();

        pickupSample2 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoses.chamberScore),/* CONTROL POINT ->>> */ new Point(autonPoses.samplePickup2CP1), new Point(autonPoses.samplePickup2Chamber))))
                .setConstantHeadingInterpolation(0)
                .build();

        goToObsZone2 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup2Chamber), new Point(autonPoses.sampleReturn2Chamber))))
                .setConstantHeadingInterpolation(0)
                .build();

        pickupSample3 = follower.pathBuilder()
                .addPath(new Path(new BezierCurve(new Point(autonPoses.chamberScore),/* CONTROL POINT ->>> */ new Point(autonPoses.samplePickup3CP1), new Point(autonPoses.samplePickup3Chamber))))
                .setConstantHeadingInterpolation(0)
                .build();

        goToObsZone3 = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.samplePickup3Chamber), new Point(autonPoses.sampleReturn3Chamber))))
                .setConstantHeadingInterpolation(0)
                .build();

        sample3ToPickup = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.sampleReturn3Chamber), new Point(autonPoses.specimenPickup))))
                .setLinearHeadingInterpolation(0, Math.toRadians(180))
                .build();

        pickupToScore = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.specimenPickup), new Point(autonPoses.chamberScore))))
                .setLinearHeadingInterpolation(Math.toRadians(180), 0)
                .build();

        scoreToPickup = follower.pathBuilder()
                .addPath(new Path(new BezierLine(new Point(autonPoses.chamberScore), new Point(autonPoses.specimenPickup))))
                .setLinearHeadingInterpolation(0, Math.toRadians(180))
                .build();

    }

    public void autonomousPathUpdate(){

        switch(pathState){

            case 0:
                follower.followPath(startToChamber);
                setPathState(1);
                break;
            case 1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the basketScore position */
                if(follower.getPose().getX() > (autonPoses.chamberScore.getX() - 1) && follower.getPose().getY() > (autonPoses.chamberScore.getY() - 1)) {
                    follower.followPath(pickupSample1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the sample1Pickup position */
                if(follower.getPose().getX() > (autonPoses.samplePickup1Chamber.getX() - 1) && follower.getPose().getY() > (autonPoses.samplePickup1Chamber.getY() - 1)) {
                    follower.followPath(goToObsZone1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.sampleReturn1Chamber.getX() - 1) && follower.getPose().getY() > (autonPoses.sampleReturn1Chamber.getY() - 1)) {
                    follower.followPath(pickupSample2, true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the sample1Pickup position */
                if(follower.getPose().getX() > (autonPoses.samplePickup2Chamber.getX() - 1) && follower.getPose().getY() > (autonPoses.samplePickup2Chamber.getY() - 1)) {
                    follower.followPath(goToObsZone2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.sampleReturn2Chamber.getX() - 1) && follower.getPose().getY() > (autonPoses.sampleReturn2Chamber.getY() - 1)) {
                    follower.followPath(pickupSample3, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the sample1Pickup position */
                if(follower.getPose().getX() > (autonPoses.samplePickup3Chamber.getX() - 1) && follower.getPose().getY() > (autonPoses.samplePickup3Chamber.getY() - 1)) {
                    follower.followPath(goToObsZone3,true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.sampleReturn3Chamber.getX() - 1) && follower.getPose().getY() > (autonPoses.sampleReturn3Chamber.getY() - 1)) {
                    follower.followPath(sample3ToPickup, true);
                    setPathState(8);
                }
                break;
            //Cycle 1
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.specimenPickup.getX() - 1) && follower.getPose().getY() > (autonPoses.specimenPickup.getY() - 1)) {
                    follower.followPath(pickupToScore, true);
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.chamberScore.getX() - 1) && follower.getPose().getY() > (autonPoses.chamberScore.getY() - 1)) {
                    follower.followPath(scoreToPickup, true);
                    setPathState(10);
                }
                break;
            //Cycle 2
            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.specimenPickup.getX() - 1) && follower.getPose().getY() > (autonPoses.specimenPickup.getY() - 1)) {
                    follower.followPath(pickupToScore, true);
                    setPathState(11);
                }
                break;
            case 11:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.chamberScore.getX() - 1) && follower.getPose().getY() > (autonPoses.chamberScore.getY() - 1)) {
                    follower.followPath(scoreToPickup, true);
                    setPathState(12);
                }
                break;
            //Cycle 3
            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.specimenPickup.getX() - 1) && follower.getPose().getY() > (autonPoses.specimenPickup.getY() - 1)) {
                    follower.followPath(pickupToScore, true);
                    setPathState(13);
                }
                break;
            case 13:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the returnSample position */
                if(follower.getPose().getX() > (autonPoses.chamberScore.getX() - 1) && follower.getPose().getY() > (autonPoses.chamberScore.getY() - 1)) {
                    follower.followPath(park, true);
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

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(autonPoses.startPoseBasket);

        buildPaths();
    }

    @Override
    public void init_loop(){
        //Looped

    }

    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }


    @Override
    public void start() {
        //Called once at the start of run, to setup basic stuff
        opmodeTimer.resetTimer();
        setPathState(0);

    }

}
