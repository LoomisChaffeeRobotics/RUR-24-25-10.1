package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.apache.commons.math3.stat.descriptive.rank.Max;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.sql.Time;
import java.util.Timer;
public class servoWork {
    ElapsedTime elapsedTime;
    public Servo Claw;
    public CRServo Tilter;
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
    int armDowning = 0;


    //    SampleMecanumDrive drive;
    public void init(HardwareMap hardwareMap) {
        elapsedTime = new ElapsedTime();
//        drive = new SampleMecanumDrive(hardwareMap);
        imu = hardwareMap.get(IMU.class,"imu");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Tilter = hardwareMap.get(CRServo.class, "Tilter");
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
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }
    public void TilterHangMoment() {
        Tilter.setPower(1.5);
    }
    public void armUp(){
        armDowning = 0;
        armTogged = false;
    }
    public void armDown(){
        armDowning = 1;
        armTogged = true;
    }
    public void armUpdate(){
        if (armDowning == 1){
            armDownPercent = Math.min(armDownPercent+0.02,1);
        } else if (armDowning == 0){
            armDownPercent = Math.max(armDownPercent-0.02,0);
        } else if (armDowning == 2) { // speciman
            if(armDownPercent <= .75){
                armDownPercent = Math.min(0.75,armDownPercent+0.02);
            } else {
                armDownPercent = Math.max(0.75,armDownPercent-0.02);
            }
        } else {
            if(armDownPercent <= .95){
                armDownPercent = Math.min(0.94,armDownPercent+0.02);
            } else {
                armDownPercent = Math.max(0.94,armDownPercent-0.02);
            }
        }
        ArmL.setPosition(0.09*(1-armDownPercent)+(.3967*armDownPercent));
        ArmR.setPosition(0.6317*(1-armDownPercent)+(0.3133*armDownPercent));

    }
    public void armSpecimen(){
        armDowning = 3;
        armTogged = true;

    }
    public void arm45(){
        armDowning = 2;
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
        LRP = 0.7;
        elapsedTime.reset();
        if(elapsedTime.time() < 1.0) {
            linearLeft.setPower(1);
            linearRight.setPower(1);
        } else {
            linearLeft.setPower(0);
            linearRight.setPower(0);
        }
    }
    public void linearUpHigh(){
        LRP = 1;
        elapsedTime.reset();
        if(elapsedTime.time() < 2.5) {
            linearLeft.setPower(1);
            linearRight.setPower(1);
        } else {
            linearLeft.setPower(0.1);
            linearRight.setPower(0.1);
        }
    }
    public void linearDownALittleBit(){
        elapsedTime.reset();
        if(elapsedTime.time() < 0.12) {
            linearLeft.setPower(-1);
            linearRight.setPower(-1);
        } else{
            linearLeft.setPower(0);
            linearRight.setPower(0);
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
        ArmL.setPosition(ArmL.getPosition()+amount);
        ArmR.setPosition(ArmR.getPosition()+amount);
    }

    public void tiltTilt(double amount){
        Tilter.setPower(amount);
    }

}