package org.firstinspires.ftc.teamcode.Stage1.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//@Disabled
@Config
@TeleOp
public class armPIDTesting extends LinearOpMode {

    private PIDController angleController;

    public static double p = .005, i = 0.05, d = 0.0002;
    public static double fAngle = .1;

    public static int angleTarget = 10;

    //Angle Motor
    static double ticks_per_rotation = 751.8;
    //TODO: UPDATE THIS VIA EMPIRICAL DATA 96 deg
    static double gear_reduction = 1/(800/((ticks_per_rotation * 360) / 96));

    private final double ticks_in_degree = (ticks_per_rotation * gear_reduction) / 360;

    private final double ticks_in_inch = ticks_in_degree / 4.409;

    private DcMotorEx angleMotorLeft;
    private DcMotorEx angleMotorRight;


    @Override
    public void runOpMode() throws InterruptedException {

        angleController = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        angleMotorRight = hardwareMap.get(DcMotorEx.class, "armAngleRight");
        angleMotorLeft = hardwareMap.get(DcMotorEx.class, "armAngleLeft");

        angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while(!isStopRequested()){

            double armAngle = angleMotorLeft.getCurrentPosition();
            angleController.setPID(p,i,d);



            // CLamping

            angleTarget = Math.max(10, Math.min(800, angleTarget));

            //Angle motor
            double anglePIDFpower = angleController.calculate(armAngle, angleTarget);
            double anglefeedForward = Math.cos(Math.toRadians(-armAngle / ticks_in_degree)) * fAngle;
            double anglePower = Math.max(-0.8, Math.min(0.8, anglePIDFpower + anglefeedForward));

            angleMotorLeft.setPower(anglePower);
            angleMotorRight.setPower(anglePower);


            telemetry.addData("Current Pos: ", armAngle);
            telemetry.addData("Current Target: ", angleTarget);
            telemetry.addData("Power", anglePower);

            telemetry.update();



        }



    }
}
