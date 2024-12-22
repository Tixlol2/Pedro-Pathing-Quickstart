//package org.firstinspires.ftc.teamcode.Intake;
//
//import androidx.annotation.NonNull;
//
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
//
//public class ClawSubsystemRoadRunner {
//
//    private Servo clawDriver;
//    private Servo clawAngle;
//    private Servo clawWrist;
//
//
//    public ClawSubsystemRoadRunner(HardwareMap hardwareMap){
//
//        this.clawDriver = hardwareMap.get(Servo.class, "clawDriver");
//        this.clawAngle = hardwareMap.get(Servo.class, "clawAngle");
//        this.clawWrist = hardwareMap.get(Servo.class, "clawWrist");
//    }
//
//    public class CloseClaw implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            clawDriver.setPosition(1);
//            return false;
//        }
//    }
//    public Action closeClaw() {
//        return new CloseClaw();
//    }
//
//    public class OpenClaw implements Action {
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            clawDriver.setPosition(0);
//            return false;
//        }
//    }
//    public Action openClaw() {
//        return new OpenClaw();
//    }
//
//
//    public class SetAngle implements Action {
//        double pos;
//        public SetAngle(double pos) {
//            this.pos = pos;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//
//            clawAngle.setPosition(pos);
//
//            return false;
//        }
//    }
//    public Action setAngle(double pos) {
//        return new SetAngle(pos);
//    }
//
//
//    public class SetWrist implements Action {
//        double pos;
//        public SetWrist(double pos) {
//            this.pos = pos;
//        }
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//
//            clawWrist.setPosition(pos);
//            return false;
//        }
//    }
//    public Action setWrist(double pos) {
//        return new SetWrist(pos);
//    }
//
//
//
//
//
//
//
//}
