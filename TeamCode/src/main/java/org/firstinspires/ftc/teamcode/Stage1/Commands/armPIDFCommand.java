package org.firstinspires.ftc.teamcode.Stage1.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Stage1.armSubsystem;

@Config
public class armPIDFCommand extends CommandBase {

    private final armSubsystem m_armSubsystem;

    private PIDController pidFController;

    public  double pAngle = .0025, iAngle = 0.08, dAngle = 0.000, fAngle = -0.08;

    public  int angleTarget = 0;



    private int armAngle;
    private double anglePower;
    private double anglePIDFpower;
    private double anglefeedForward;

    private final double ticks_in_degree = (751.8 * 4) / 360;

    private final double ticks_in_inch = (537.7 / 112) / 25.4;

    private PIDController pidController;

    public  double pExtend = 0.008, iExtend = 0, dExtend = 0, fExtend = 0;

    public  int extendTarget = 0;



    private int extendPos;
    private double powerExtension;
    private double PIDFpowerExtend;

    public armPIDFCommand(armSubsystem subsystem, int angleTarget, int extendTarget) {
        m_armSubsystem = subsystem;
        addRequirements(m_armSubsystem);
        pidFController = new PIDController(pAngle, iAngle, dAngle);
        pidController = new PIDController(pExtend, iExtend, dExtend);
        this.angleTarget = angleTarget;
        this.extendTarget = extendTarget;

    }

    @Override
    public void initialize() {
//        m_armSubsystem.setArmTarget(angleTarget);
//        m_armSubsystem.setExtendTarget(extendTarget);
    }

    @Override
    public void execute() {
        if(angleTarget >= -15){
            angleTarget = -15;
        } else if (angleTarget <= -550){angleTarget = -550;}
        //Angle motor
        pidFController.setPID(pAngle, iAngle, dAngle);
//        armAngle = m_armSubsystem.angleMotor.getCurrentPosition();
        anglePIDFpower = pidFController.calculate(armAngle, angleTarget);
        anglefeedForward = Math.cos(Math.toRadians(armAngle / ticks_in_degree)) * fAngle;
        anglePower = anglePIDFpower + anglefeedForward;
        if(anglePower > .8 ){
            anglePower = .8;
        } else if (anglePower < -.8){anglePower = -.8;}
//        m_armSubsystem.angleMotor.setPower(anglePower);


        //Extension motor
        if(extendTarget >= -50){
            extendTarget = -50;
        } else if (extendTarget <= -3400){angleTarget = -3400;}
        pidController.setPID(pExtend, iExtend, dExtend);
//        extendPos = m_armSubsystem.extenderMotor.getCurrentPosition();
        PIDFpowerExtend = pidController.calculate(extendPos, extendTarget);

        powerExtension = PIDFpowerExtend;
        if(powerExtension > .8 ){
            powerExtension = .8;
        } else if (powerExtension < -.8){powerExtension = -.8;}
//        m_armSubsystem.extenderMotor.setPower(powerExtension);


    }

    @Override
    public boolean isFinished() {

        if ((armAngle >= angleTarget - 10 && armAngle <= angleTarget + 10) && (extendPos >= extendTarget - 30 && extendPos <= angleTarget + 30)){return true;}
        else{return false;}


    }
}
