package org.firstinspires.ftc.teamcode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class extendTest extends LinearOpMode {

    DcMotorEx upMotor;
    DcMotorEx downMotor;
    double power;

    @Override
    public void runOpMode() throws InterruptedException {

        upMotor = hardwareMap.get(DcMotorEx.class, "armExtendUp");
        downMotor = hardwareMap.get(DcMotorEx.class, "armExtendDown");
        upMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        downMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        while (opModeInInit()){




        }

        while(opModeIsActive()){

            power = gamepad1.left_stick_y;

            upMotor.setPower(-power);
            downMotor.setPower(-power);

            telemetry.addData("Up Motor Ticks", upMotor.getCurrentPosition());
            telemetry.addData("Down Motor Ticks", downMotor.getCurrentPosition());
            telemetry.update();

        }






    }
}
