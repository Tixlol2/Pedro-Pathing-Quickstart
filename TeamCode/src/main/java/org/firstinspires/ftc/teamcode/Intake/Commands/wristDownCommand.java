package org.firstinspires.ftc.teamcode.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Intake.clawSubsystem;

public class wristDownCommand extends CommandBase {

    private final clawSubsystem m_clawSubsystem;

    public wristDownCommand(clawSubsystem subsystem){

        m_clawSubsystem = subsystem;

    }

    @Override
    public void initialize(){
        m_clawSubsystem.setAnglePosition(1);
    }

    @Override
    public boolean isFinished() {
        if (m_clawSubsystem.driverOfClaw.getPosition() == 1) {
            return true;
        } else {
            return false;
        }

    }

}
