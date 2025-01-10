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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.sql.Time;
import java.util.Timer;

public class servoWork {
    ElapsedTime elapsedTime;
    Servo Claw;
    Servo ArmR;
    Servo ArmL;
//    DcMotor Tilter;
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
//    SampleMecanumDrive drive;
    public void init(HardwareMap hardwareMap) {
        elapsedTime = new ElapsedTime();
//        drive = new SampleMecanumDrive(hardwareMap);
        imu = hardwareMap.get(IMU.class,"imu");
        Claw = hardwareMap.get(Servo.class, "Claw");
        ArmR = hardwareMap.get(Servo.class, "ArmR");
        ArmR.setDirection(REVERSE);
        ArmL = hardwareMap.get(Servo.class, "ArmL");
        ArmL.setDirection(FORWARD);
        Extender = hardwareMap.get(CRServo.class, "Extender");
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Tilter = hardwareMap.get(DcMotor.class, "Tilter");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public double armUp(){
        ArmL.setPosition(ArmL.getPosition()+0.001);
//        ArmL.setPosition(0.34);
//        ArmR.setPosition(.68);
        armTogged = false;
        return ArmL.getPosition();
    }
    public double armDown(){
        ArmL.setPosition(ArmL.getPosition()-0.001);
//        ArmL.setPosition(0.8);
//        ArmR.setPosition(0.34);
        armTogged = true;
        return ArmL.getPosition();
    }
    public void armSpecimen(){
        ArmL.setPosition(0.41);
        ArmR.setPosition(0.36);
        armTogged = true;
    }
    public void armToggle(){

        if(armTogged){
            armUp();
        } else{
            armDown();
        }
    }

    public void linearUpHigh(double amnt){
        LRP = amnt;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
//        linearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void linearUpSubmersible(){
        LRP = 0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
//        linearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    public void linearDownFull(){
        LRP = -0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
//        linearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void linearNeutral(){
        LRP = 0.1;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);

    }
    public void clawOpen(){
        Claw.setPosition(1);
        clawTogged = true;
    }
    public void clawClosed(){
        Claw.setPosition(0);
        clawTogged = false;
    }
    public void clawToggle(){
        if (clawTogged){
            clawClosed();
        } else{
            clawOpen();
        }
    }
    public int clawPort(){
        Claw.getPortNumber();
        return Claw.getPortNumber();
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
//    public void tilterUp() {
//        Tilter.setPower(0.5);
//    }
//    public void tilterDown() {
//        Tilter.setPower(-0.5);
//    }

}
