package org.firstinspires.ftc.teamcode.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Intake.ClawSubsystem;

public class wristUpCommand extends CommandBase {

    private final ClawSubsystem m_clawSubsystem;

    public wristUpCommand(ClawSubsystem subsystem){

        m_clawSubsystem = subsystem;

    }

    @Override
    public void initialize(){
        m_clawSubsystem.setAnglePosition(0);
    }

    @Override
    public boolean isFinished() {
        if (m_clawSubsystem.driverOfClaw.getPosition() == 0) {
            return true;
        } else {
            return false;
        }

    }

}
