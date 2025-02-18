package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class servoWork {
    ElapsedTime elapsedTime;
    public Servo Claw;
    public DcMotor Tilter;
    public Servo ArmR;
    public Servo ArmL;
    CRServo Extender;
    DcMotor linearRight;
    DcMotor linearLeft;
    boolean armTogged = true;
    boolean armTouched;
    boolean clawTogged = false;
    boolean linearToggedHigh = true;
    boolean linearToggedMedium = true;
    double LRP = 0; //linear slide power, LRP because Millen named that
    IMU imu;
    public double armDownPercent;
    public double armDownPrevious = 0;



    public enum armState {
        UP,
        SUBMERSIBLE_EXIT,
        SPECIMEN,
        SAMPLE,
        HALF,
        NULL
    }
    public armState armDowning = armState.UP;


    //    SampleMecanumDrive drive;
    public void init(HardwareMap hardwareMap) {
        elapsedTime = new ElapsedTime();
//        drive = new SampleMecanumDrive(hardwareMap);
        Claw = hardwareMap.get(Servo.class, "Claw");
        Tilter = hardwareMap.get(DcMotor.class, "Tilter");
        ArmR = hardwareMap.get(Servo.class, "ArmR");
        ArmR.setDirection(REVERSE);
        ArmL = hardwareMap.get(Servo.class, "ArmL");
        ArmL.setDirection(REVERSE);
        Extender = hardwareMap.get(CRServo.class, "Extender");
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    boolean timerTestStart = false;
    public double timerTestStart(){
        if (!timerTestStart){
            elapsedTime.reset();
            timerTestStart = true;
        }
        return elapsedTime.time();
    }
    public void TilterHangMoment() {
        Tilter.setPower(1);
    }
    public void armUp(){
        armDowning = armState.UP;
        armTogged = false;
    }
    public void armDown(){
        armDowning = armState.SAMPLE;
        armTogged = true;
    }
    public void armUpAuto(){
        ArmL.setPosition(0.3883);
        ArmR.setPosition(0.29);
    }
    public void armDownAuto(){
        ArmL.setPosition(0.6744);
        ArmR.setPosition(0.);
    }
    public void armUpdate(){
        if (armDowning == armState.SAMPLE){
            armDownPercent = Math.min(armDownPercent+0.02,1);

        } else if (armDowning == armState.UP){
            armDownPercent = Math.max(armDownPercent-0.02,0);
        } else if (armDowning == armState.SUBMERSIBLE_EXIT) { // used for getting out of submersible when facing chambers
            if(armDownPercent <= .75){
                armDownPercent = Math.min(0.75,armDownPercent+0.02);
            } else {
                armDownPercent = Math.max(0.75,armDownPercent-0.02);
            }
        } else if (armDowning == armState.SPECIMEN) { //specimen
            if(armDownPercent <= .94){
                armDownPercent = Math.min(0.94,armDownPercent+0.02);
            } else {
                armDownPercent = Math.max(0.94,armDownPercent-0.02);
            }

//            TODO: delete this.
//        } else if (armDowning == armState.?) {
//            if(armDownPercent <= -.15){
//                armDownPercent = Math.min(-0.15,armDownPercent-0.01);
//            }
        }
        ArmL.setPosition(0.3883*(1-armDownPercent)+(0.6594*armDownPercent));
        ArmR.setPosition(0.29*(1-armDownPercent)+(0.015*armDownPercent));
    }
    public void waitAuto(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.seconds() < seconds) {

        }
    }
    public void armUpdateAuto(){
        if (armDowning == armState.SAMPLE){
            armDownPercent = Math.min(armDownPercent+0.01,1);

        } else if (armDowning == armState.UP){
            armDownPercent = Math.max(armDownPercent-0.01,0);
        } else if (armDowning == armState.SUBMERSIBLE_EXIT) { // used for getting out of submersible when facing chambers
            if(armDownPercent <= .75){
                armDownPercent = Math.min(0.75,armDownPercent+0.01);
            } else {
                armDownPercent = Math.max(0.75,armDownPercent-0.01);
            }
        } else if (armDowning == armState.SPECIMEN) { //specimen
            if(armDownPercent <= .94){
                armDownPercent = Math.min(0.94,armDownPercent+0.01);
            } else {
                armDownPercent = Math.max(0.94,armDownPercent-0.01);
            }

//            TODO: delete this.
//        } else if (armDowning == armState.?) {
//            if(armDownPercent <= -.15){
//                armDownPercent = Math.min(-0.15,armDownPercent-0.01);
//            }
        }
        ArmL.setPosition(0.3883*(1-armDownPercent)+(0.6594*armDownPercent));
        ArmR.setPosition(0.29*(1-armDownPercent)+(0.015*armDownPercent));
    }
    public boolean isArmRunning() {
        if (armDownPrevious==armDownPercent){
            armDownPrevious = armDownPercent;
            return false;
        }
        armDownPrevious = armDownPercent;
        return true;
    }
    public void armSpecimen(){
        armDowning = armState.SPECIMEN;
        armTogged = true;

    }
    public void arm45(){ //only used for auto
        armDowning = armState.HALF;
    }
    public void armSubmersible(){
        armDowning = armState.SUBMERSIBLE_EXIT;
    }
    public void armHang() {
        armDowning = armState.UP;
    }
    public void armToggle(){

        if(armTogged){
            armUp();
        } else{
            armDown();
        }
    }
    public double armRightManual(double amount){
        ArmR.setPosition(ArmR.getPosition()+amount);
        return ArmR.getPosition();
    }
    public double armLeftManual(double amount){
        ArmL.setPosition(amount);
        return ArmL.getPosition();
    }
    public void linearUpSubmersible(){
        elapsedTime.reset();
        if(elapsedTime.time() < 1.0) {
            linearLeft.setPower(1);
            linearRight.setPower(1);
        } else {
            linearLeft.setPower(0.12);
            linearRight.setPower(0.12);
        }
    }
    public void linearUpHigh(){
        elapsedTime.reset();
        if(elapsedTime.time() < 2.5) {
            linearLeft.setPower(1);
            linearRight.setPower(1);
        } else {
            linearLeft.setPower(0.12);
            linearRight.setPower(0.12);
        }
    }
    public void linearDownALittleBit(){
        elapsedTime.reset();
        if(elapsedTime.time() < 0.12) {
            linearLeft.setPower(-1);
            linearRight.setPower(-1);
        } else{
            linearLeft.setPower(0.12);
            linearRight.setPower(0.12);
        }
    }
    public void linearDownFullFromSubmersible(){
        elapsedTime.reset();
        if(elapsedTime.time() < 0.6) {
            linearLeft.setPower(-1);
            linearRight.setPower(-1);
        } else {
            linearLeft.setPower(0);
            linearRight.setPower(0);
        }
    }
    public void linearDownFullFromHigh(){
        elapsedTime.reset();
        if(elapsedTime.time() < 1.6) {
            linearLeft.setPower(-1);
            linearRight.setPower(-1);
        } else {
            linearLeft.setPower(0);
            linearRight.setPower(0);
        }
    }
    public void clawOpen(){
        Claw.setPosition(1);
        clawTogged = true;
    }
    public void clawClosed(){
        Claw.setPosition(0.3817);
        clawTogged = false;
    }
    public void clawToggle(){
        if (clawTogged){
            clawClosed();
        } else{
            clawOpen();
        }
    }
    public void liftLift(double power){
        linearLeft.setPower(power);
        linearRight.setPower(power);
    }
    public void clawManual(double amount){
        Claw.setPosition(Claw.getPosition()+amount);
    }
    public void extenderForward(){
        Extender.setPower(1);
    }
    public void extenderBack(){
        Extender.setPower(-1);
    }
    public void extenderNeutral(){
        Extender.setPower(0);
    }
    public void armDoubleManual(double amount) {
        ArmL.setPosition(ArmL.getPosition()-amount);
        ArmR.setPosition(ArmR.getPosition()+amount);
    }

    public void tiltTilt(double amount){
        Tilter.setPower(amount);
    }

}