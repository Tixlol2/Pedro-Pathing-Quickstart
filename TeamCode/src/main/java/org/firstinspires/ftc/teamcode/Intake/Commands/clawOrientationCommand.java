package org.firstinspires.ftc.teamcode.Intake.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Intake.ClawSubsystem;

public class clawOrientationCommand extends CommandBase {

    private final ClawSubsystem m_clawSubsystem;
    private final double targetPosition;

    public clawOrientationCommand(ClawSubsystem subsystem, double target){
        m_clawSubsystem = subsystem;
        targetPosition = target;
    }

    @Override
    public void initialize(){
        m_clawSubsystem.setAnglePosition(targetPosition);

    }

    @Override
    public boolean isFinished() {
        if (m_clawSubsystem.wristOfClaw.getPosition() == targetPosition) {
            return true;
        } else {
            return false;
        }

    }

}
