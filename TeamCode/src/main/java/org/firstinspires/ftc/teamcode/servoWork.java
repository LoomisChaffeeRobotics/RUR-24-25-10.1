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
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void armUp(){
        ArmL.setPosition(0.73);
        ArmR.setPosition(.68);
        armTogged = false;
    }
    public void armDown(){
        ArmL.setPosition(.39);
        ArmR.setPosition(0.34);
        armTogged = true;
    }
    public void armSpecimen(){
        ArmL.setPosition(0.43);
        ArmR.setPosition(0.38);
        armTogged = true;
    }
    public void armToggle(){

        if(armTogged){
            armUp();
        } else{
            armDown();
        }
    }
    public double armLeftManual(double amount){
        ArmR.setPosition(ArmR.getPosition()+amount);
        return ArmR.getPosition();
    }

    public void linearUpHigh(){
        LRP = 0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        linearRight.setTargetPosition(60);
        linearLeft.setTargetPosition(60);
//        linearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void linearUpSubmersible(){
        LRP = 0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        elapsedTime.reset();
        linearRight.setTargetPosition(50);
        linearLeft.setTargetPosition(50);
//        linearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    public void linearDownFull(){
        LRP = -0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        elapsedTime.reset();
        linearLeft.setTargetPosition(4);
        linearRight.setTargetPosition(4);
//        linearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        linearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void clawOpen(){
        Claw.setPosition(0.339);
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

    public void clawManual(double amount){
        Claw.setPosition(Claw.getPosition()+amount);
    }
    public void extenderForward(){
        Extender.setPower(-0.5);
    }
    public void extenderBack(){
        Extender.setPower(0.2);
    }
    public void extenderNeutral(){
        Extender.setPower(0);
    }


}
