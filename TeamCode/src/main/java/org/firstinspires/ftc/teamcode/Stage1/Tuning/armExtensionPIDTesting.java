package org.firstinspires.ftc.teamcode.Stage1.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class armExtensionPIDTesting extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.008, i = 0.05, d = 0;


    public static int target = 0;

    private final double ticks_in_degree = 537.7;

    private DcMotorEx arm_motor_up;
    private DcMotorEx arm_motor_down;


    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm_motor_up = hardwareMap.get(DcMotorEx.class, "armExtendUp");
        arm_motor_down = hardwareMap.get(DcMotorEx.class, "armExtendDown");

        arm_motor_up.setDirection(DcMotorSimple.Direction.REVERSE);
        arm_motor_down.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(!isStopRequested()){
            if(target <= 10){target = 10;}
            if (target >= 3600){target = 3600;}
            controller.setPID(p, i, d);
            int armPos = arm_motor_up.getCurrentPosition();
            double pidPower = controller.calculate(armPos, target);




            arm_motor_down.setPower(pidPower);
            arm_motor_up.setPower(pidPower);

            telemetry.addData("Current Pos: ", armPos);
            telemetry.addData("Current Target: ", target);
            telemetry.update();



        }



    }
}
