package org.firstinspires.ftc.teamcode.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Intake.ClawSubsystem;

public class clawCloseCommand extends CommandBase {

    private final ClawSubsystem m_clawSubsystem;

    public clawCloseCommand(ClawSubsystem subsystem) {

        m_clawSubsystem = subsystem;

    }

    @Override
    public void initialize() {
        m_clawSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        if (m_clawSubsystem.driverOfClaw.getPosition() == m_clawSubsystem.closed) {
            return true;
        } else {
            return false;
        }

    }
}
