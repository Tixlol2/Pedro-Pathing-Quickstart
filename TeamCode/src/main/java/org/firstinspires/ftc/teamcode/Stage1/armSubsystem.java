package org.firstinspires.ftc.teamcode.Stage1;



import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class armSubsystem extends SubsystemBase {

    //Motor used to change the angle of the arm
    private final DcMotorEx extenderMotorUp;
    private final DcMotorEx extenderMotorDown;
    private final DcMotorEx angleMotorLeft;
    private final DcMotorEx angleMotorRight;




    private static final double pAngle = 0.005, iAngle = 0.0, dAngle = 0;
    private static double fAngle = 0.1;


    //Angle Motor
    static double ticks_per_rotation = 751.8;
    //TODO: UPDATE THIS VIA EMPIRICAL DATA 96 deg
    static double gear_reduction = 1/(800/((ticks_per_rotation * 360) / 96));

    private static final double ticks_in_degree = (ticks_per_rotation * gear_reduction) / 360;


    //Extension Motor
    static double ticks_per_rotation_ext = 537.7;

    private static final double ticks_in_inch = ticks_per_rotation_ext / (112 / 25.4);


    private static double pExtend = 0.008, iExtend = 0.05, dExtend = 0, fExtend = 0;


    private static int anglePos;
    private static int angleTarget;
    private int extPos;
    private int extTarget;

    private static final int angleMax = 800;
    private static final int angleMin = 10;
    private static final int extMin = 10;
    private static final int extMax = 3600;

    private static final PIDController angleController = new PIDController(pAngle, iAngle, dAngle);
    private static final PIDController extendController = new PIDController(pExtend, iExtend, dExtend);




    public armSubsystem(final HardwareMap hmap, final String extensionLeft, final String extensionRight, final String angleUp, final String angleDown){
        extenderMotorUp = hmap.get(DcMotorEx.class, extensionLeft);
        angleMotorLeft = hmap.get(DcMotorEx.class, angleUp);

        extenderMotorDown = hmap.get(DcMotorEx.class, extensionRight);
        angleMotorRight = hmap.get(DcMotorEx.class, angleDown);

        extenderMotorUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angleMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extenderMotorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        extenderMotorDown.setDirection(DcMotorSimple.Direction.REVERSE);
        extenderMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);



        angleController.setPID(pAngle, iAngle, dAngle);
        extendController.setPID(pExtend, iExtend, dExtend);


    }

    public armSubsystem(final HardwareMap hmap,final String extensionUp, final String extensionDown, final String angleLeft, final String angleRight, final double pAngle, final double iAngle, final double dAngle, final double fAngle, final double pExtend, final double iExtend, final double dExtend){
        extenderMotorUp = hmap.get(DcMotorEx.class, extensionUp);
        angleMotorLeft = hmap.get(DcMotorEx.class, angleLeft);

        extenderMotorDown = hmap.get(DcMotorEx.class, extensionDown);
        angleMotorRight = hmap.get(DcMotorEx.class, angleRight);

        extenderMotorUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        angleMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extenderMotorDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        angleMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        extenderMotorDown.setDirection(DcMotorSimple.Direction.REVERSE);
        extenderMotorUp.setDirection(DcMotorSimple.Direction.REVERSE);



        angleController.setPID(pAngle, iAngle, dAngle);
        extendController.setPID(pExtend, iExtend, dExtend);

        this.fAngle = fAngle;


    }

    // ----------------
    // Setters
    // ----------------
    public void setPos(Vector2d armPos) {
        angleTarget = (int) (Math.toDegrees(Math.atan(armPos.getY()/armPos.getX())) * ticks_in_degree);
        extTarget = (int) ((Math.sqrt(armPos.getX()*armPos.getX() + armPos.getY()*armPos.getY()) -18)* ticks_in_inch);
    }
    public void setPos(int ext, int angle) {
        angleTarget = (int) (angle * ticks_in_degree);
        extTarget = (int) ((ext -18)* ticks_in_inch);
    }
    // ----------------
    // Getters
    // ----------------
    public int getAngleTarget(){return angleTarget;}
    public double getAngleTargetDG(){return (angleTarget / ticks_in_degree);}

    public int getAnglePos(){return angleMotorLeft.getCurrentPosition();}
    public double getAnglePosDEG(){return (angleMotorLeft.getCurrentPosition() / ticks_in_degree);}

    public int getExtTarget(){return extTarget;}
    public double getExtTargetIN(){return (extTarget / ticks_in_inch);}

    public int getExtenderPos(){return extenderMotorUp.getCurrentPosition();}
    public double getExtenderPosIN(){return (extenderMotorUp.getCurrentPosition() / ticks_in_inch);}

    public double getX(){return (extenderMotorUp.getCurrentPosition()/ticks_in_inch +18)* Math.cos(Math.toRadians(angleMotorLeft.getCurrentPosition()/ticks_in_degree));}
    public double getY(){return (extenderMotorUp.getCurrentPosition()/ticks_in_inch +18) * Math.sin(Math.toRadians(angleMotorLeft.getCurrentPosition()/ticks_in_degree));}
    // ----------------
    // Calculations
    // ----------------


    public void update(int setAngleTarget, int setExtendTarget) {
        angleTarget = setAngleTarget;
        extTarget = setExtendTarget;
        double anglePower;
        double extendPower;
        double anglefeedForward;
        double anglePIDFpower;
        int extendPos;

        double armAngle = angleMotorLeft.getCurrentPosition();
        double armExt = extenderMotorUp.getCurrentPosition();


        double anglePIDFPower;
        armAngle = angleMotorLeft.getCurrentPosition();

        // CLamping

        angleTarget = Math.max(angleMin, Math.min(angleMax, angleTarget));
        extTarget = (int) Math.max(extMin, Math.min(Math.min(extMax, extMax - ((extMax - 16*ticks_in_inch) * Math.cos(Math.toRadians(armAngle / ticks_in_degree)))), extTarget));

        //Angle motor
        //angleController.setPID(pAngle,iAngle,dAngle);
        anglePIDFpower = angleController.calculate(armAngle, angleTarget);
        anglefeedForward = Math.cos(Math.toRadians(armAngle / ticks_in_degree)) * fAngle;
        anglePower = Math.max(-0.4, Math.min(0.8, anglePIDFpower + anglefeedForward));

        angleMotorLeft.setPower(anglePower);
        angleMotorRight.setPower(anglePower);

        //Extension motor
        //extendController.setPID(pExtend,iExtend,dExtend);
        extendPower = Math.max(-0.8, Math.min(0.8, extendController.calculate(extenderMotorUp.getCurrentPosition(), extTarget)));

        extenderMotorUp.setPower(extendPower);
        extenderMotorDown.setPower(extendPower);
    }
    public void update() {
        double anglePower;
        double extendPower;
        double anglefeedForward;
        double anglePIDFpower;
        int extendPos;

        double armAngle = angleMotorLeft.getCurrentPosition();
        double armExt = extenderMotorUp.getCurrentPosition();


        double anglePIDFPower;
        armAngle = angleMotorLeft.getCurrentPosition();

        // CLamping

        angleTarget = Math.max(angleMin, Math.min(angleMax, angleTarget));
        extTarget = (int) Math.max(extMin, Math.min(Math.min(extMax, extMax - ((extMax - 26*ticks_in_inch) * Math.cos(Math.toRadians(armAngle / ticks_in_degree)))), extTarget));

        //Angle motor
        //angleController.setPID(pAngle,iAngle,dAngle);
        anglePIDFpower = angleController.calculate(armAngle, angleTarget);
        anglefeedForward = Math.cos(Math.toRadians(armAngle / ticks_in_degree)) * fAngle;
        anglePower = Math.max(-0.4, Math.min(0.8, anglePIDFpower + anglefeedForward));

        angleMotorLeft.setPower(anglePower);
        angleMotorRight.setPower(anglePower);

        //Extension motor
        //extendController.setPID(pExtend,iExtend,dExtend);
        extendPower = Math.max(-0.8, Math.min(0.8, extendController.calculate(extenderMotorUp.getCurrentPosition(), extTarget)));

        extenderMotorUp.setPower(extendPower);
        extenderMotorDown.setPower(extendPower);
    }
}
