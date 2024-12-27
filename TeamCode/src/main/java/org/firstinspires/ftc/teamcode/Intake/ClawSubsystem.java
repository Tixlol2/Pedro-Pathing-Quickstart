package org.firstinspires.ftc.teamcode.Intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase {

    public static Servo angleOfClaw = null;
    public static Servo driverOfClaw = null;
    public static Servo wristOfClaw = null;

    public static final double open = 0;
    public final double closed = 1;


    //hMap is understandable, name is the name of the servo used
    public ClawSubsystem(final HardwareMap hMap, final String angleName, final String openCloseName, final String wristName){
        wristOfClaw = hMap.get(Servo.class, wristName);
        angleOfClaw = hMap.get(Servo.class, angleName);
        driverOfClaw = hMap.get(Servo.class, openCloseName);
    }


    public ClawSubsystem(final HardwareMap hMap){
//"clawAngle", "clawDriver", "clawWrist"
        wristOfClaw = hMap.get(Servo.class, "clawAngle");
        angleOfClaw = hMap.get(Servo.class, "clawWrist");
        driverOfClaw = hMap.get(Servo.class, "clawDriver");
    }

    public static void open(){


        driverOfClaw.setPosition(open);
    }

    public static void close(){
        driverOfClaw.setPosition(closed);
    }
    //Using a dorect connection, this should hold up
    public static void setAnglePosition(double position){angleOfClaw.setPosition(Math.max(0, Math.min(1, position)));}
    public static void setWristPosition(double position){wristOfClaw.setPosition(position);}
}
