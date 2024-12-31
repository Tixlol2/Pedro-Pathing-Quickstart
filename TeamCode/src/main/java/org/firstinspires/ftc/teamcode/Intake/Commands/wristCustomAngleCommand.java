package org.firstinspires.ftc.teamcode.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Intake.ClawSubsystem;

public class wristCustomAngleCommand extends CommandBase {

    private final ClawSubsystem m_clawSubsystem;
    private final double targetPosition;

    public wristCustomAngleCommand(ClawSubsystem subsystem, double target){

        m_clawSubsystem = subsystem;
        targetPosition = target;
    }

    @Override
    public void initialize(){
        m_clawSubsystem.setAnglePosition(targetPosition);

    }

    @Override
    public boolean isFinished() {
        if (m_clawSubsystem.driverOfClaw.getPosition() == targetPosition) {
            return true;
        } else {
            return false;
        }

    }

}
