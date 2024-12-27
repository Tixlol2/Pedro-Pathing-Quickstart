package org.firstinspires.ftc.teamcode.Stage1;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class armSubsystemPedro extends armSubsystem{




    public armSubsystemPedro(HardwareMap hardwareMap){
        super(hardwareMap, "armExtendUp", "armExtendDown", "armAngleLeft", "armAngleRight", 0.005, 0, 0, 0.1, 0.008, 0.05, 0);
    }





    


}
