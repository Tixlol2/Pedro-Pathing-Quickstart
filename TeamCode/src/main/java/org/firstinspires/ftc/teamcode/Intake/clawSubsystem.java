package org.firstinspires.ftc.teamcode.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class clawSubsystem extends SubsystemBase {

    public final Servo angleOfClaw;
    public final Servo driverOfClaw;
    public final Servo wristOfClaw;

    public final double open = 0;
    public final double closed = 1;


    //hMap is understandable, name is the name of the servo used
    public clawSubsystem(final HardwareMap hMap, final String angleName, final String openCloseName, final String wristName){
        wristOfClaw = hMap.get(Servo.class, wristName);
        angleOfClaw = hMap.get(Servo.class, angleName);
        driverOfClaw = hMap.get(Servo.class, openCloseName);
    }

    public clawSubsystem(final HardwareMap hMap){
//"clawAngle", "clawDriver", "clawWrist"
        wristOfClaw = hMap.get(Servo.class, "clawAngle");
        angleOfClaw = hMap.get(Servo.class, "clawWrist");
        driverOfClaw = hMap.get(Servo.class, "clawDriver");
    }

    public void open(){
        driverOfClaw.setPosition(open);
    }

    public void close(){
        driverOfClaw.setPosition(closed);
    }
    //Using a dorect connection, this should hold up
    public void setAnglePosition(double position){angleOfClaw.setPosition(Math.max(0, Math.min(1, position)));}
    public void setWristPosition(double position){wristOfClaw.setPosition(position);}
}
