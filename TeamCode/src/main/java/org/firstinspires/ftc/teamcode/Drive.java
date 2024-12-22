package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Intake.clawSubsystem;
import org.firstinspires.ftc.teamcode.Stage1.armSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


@TeleOp(name="Drive", group = "Drive")
public class Drive extends LinearOpMode {

    //Class def


    double gp2Deflator;
    double gp1Deflator;

    int angleTarget = 10;
    int extendTarget = 10;
    double clawTarget = 1;
    double clawWrist = 0.5;




    boolean driveCentric;
    Follower follower;


    private PIDController angleController;
    private PIDController extendController;




    public  double pAngle = .005, iAngle = 0, dAngle = 0.000, fAngle = .1;
    public static double pExtend = 0.008, iExtend = 0, dExtend = 0;



    @Override
    public void runOpMode() throws InterruptedException {


        //During Initialization:
        angleController = new PIDController(pAngle, iAngle, dAngle);
        extendController = new PIDController(pExtend, iExtend, dExtend);

        follower = new Follower(hardwareMap);


        //ll3a = hardwareMap.get(Limelight3A.class, "LL3a");



        //commandScheduler = CommandScheduler.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //hMap, name of servo used for claw
        clawSubsystem clawSubsystem = new clawSubsystem(hardwareMap, "clawAngle", "clawDriver", "clawWrist");
        //hMap, name of motor used to change the EXTENSION HEIGHT of the arm/slides
        armSubsystem armSubsystem = new armSubsystem(hardwareMap, "armExtendUp", "armExtendDown", "armAngleLeft", "armAngleRight");
//        armPIDFCommand armPIDFCommand = new armPIDFCommand(armSubsystem, 0,0 );



        waitForStart();



       follower.startTeleopDrive();



        while (opModeIsActive()) {
            //-----------------------------
            //Input
            // ----------------------------

            gp2Deflator = gamepad2.left_bumper ? 0.5 : 1;
            gp1Deflator = gamepad1.left_bumper && gamepad1.right_bumper ? 0.5 : gamepad1.left_bumper ? 0.7 : 1;

            if (gamepad1.b) {
                driveCentric = false;
            } else if (gamepad1.a) {
                driveCentric = true;
            }
            //Testing clawSubsystem
            if (gamepad2.b) {
                clawSubsystem.close();
            } else if (gamepad2.a) {
                clawSubsystem.open();
            }
            if (gamepad2.x) {
                clawTarget = 1;
            } else if (gamepad2.y) {
                clawTarget = 0;
            }
            if (gamepad2.dpad_right ){clawSubsystem.setWristPosition(1);}
            else if (gamepad2.dpad_up){clawSubsystem.setWristPosition(0);}

            //Testing armSubsystem
            clawTarget += (Math.pow(gamepad2.left_trigger + -gamepad2.right_trigger,3) * 0.025 * gp2Deflator);
            clawWrist = gamepad2.dpad_right ? 1 : gamepad2.dpad_up ? 0.5 : clawWrist;


            //Extend to basket
            if (gamepad2.dpad_left) {
                armSubsystem.setPos(46,90);
                clawSubsystem.setAnglePosition(0.5);
                clawSubsystem.setWristPosition(0.5);
                angleTarget = armSubsystem.getAngleTarget();
                extendTarget = armSubsystem.getExtTarget();
//                wait(500);
//                clawSubsystem.open();
//                wait(100);
//                clawSubsystem.close();
//                armSubsystem.setPos(0, 0);
//                clawSubsystem.setAnglePosition(0);
//                clawSubsystem.setWristPosition(0);
            //
            } else if (gamepad2.dpad_down) {
                armSubsystem.setPos(22,0);
                clawTarget = 0;
                clawSubsystem.setWristPosition(0.5);
                clawSubsystem.open();
                angleTarget = armSubsystem.getAngleTarget();
                extendTarget = armSubsystem.getExtTarget();
            //Prepare to score specimen
            } else if (gamepad2.left_stick_button) {
                armSubsystem.setPos(0,45);
                angleTarget = armSubsystem.getAngleTarget();
                extendTarget = armSubsystem.getExtTarget();
            }

            angleTarget += (int) (Math.pow(gamepad2.left_stick_y, 3) * -16 * gp2Deflator);
            extendTarget += (int) (Math.pow(gamepad2.right_stick_y, 3) * -120 * gp2Deflator);


            // ----------------------------
            // Telemetry
            // ----------------------------

            telemetry.addData("Current Angle in Ticks: ", armSubsystem.getAnglePos());
            telemetry.addData("Current Angle Target in Ticks: ", angleTarget);


            telemetry.addData("Current Extension in Ticks: ", armSubsystem.getExtenderPos());
            telemetry.addData("Current Extension Target in Ticks: ", extendTarget);



            telemetry.addData("Arm Angle: ", armSubsystem.getAnglePosDEG());
            telemetry.addData("Arm extension: ", armSubsystem.getExtenderPosIN());

            telemetry.addData("Arm subsystem Angle Target:", armSubsystem.getAngleTarget());
            telemetry.addData("Arm subsystem Extension Target:", armSubsystem.getExtTarget());

            telemetry.addData("X: ", armSubsystem.getX());
            telemetry.addData("Y: ", armSubsystem.getY());


            telemetry.addLine("Don't Crash!");
            telemetry.addData("Driver Centric?", driveCentric);



            // ---------------
            // Motor Calculations
            // ----------------



            armSubsystem.update(angleTarget,extendTarget);
            // ----------------------------
            // Updaters
            // ----------------------------
            clawTarget = Math.max(armSubsystem.getExtenderPos() < 30 ? 0.4: 0, Math.min(1, clawTarget));
            clawWrist = Math.max(0, Math.min(1, clawWrist));
            clawSubsystem.setAnglePosition(clawTarget);
            clawSubsystem.setWristPosition(clawWrist);
            follower.updatePose();
            follower.setTeleOpMovementVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, true);

            angleTarget = armSubsystem.getAngleTarget();
            extendTarget = armSubsystem.getExtTarget();

            telemetry.addLine("Lock In 🔥 🔥 🔥");
            telemetry.addLine("Improvement Is The First Step to Success");
            telemetry.update();
        }




    }
}
