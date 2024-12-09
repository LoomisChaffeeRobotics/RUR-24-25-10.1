package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.sql.Time;

@TeleOp
public class servoWork extends OpMode{
    Servo Claw;
    Servo ArmR;
    Servo ArmL;
    Servo Extender;
    DcMotor linearRight;
    DcMotor linearLeft;
    boolean armTogged = true;
    boolean clawTogged = true;
    double LRP = 0; //linear slide power, LRP because Millen named that
    IMU imu;
    SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        imu = hardwareMap.get(IMU.class,"imu");
        Claw = hardwareMap.get(Servo.class, "Claw");
        ArmR = hardwareMap.get(Servo.class, "ArmR");
        ArmR.setDirection(REVERSE);
        ArmL = hardwareMap.get(Servo.class, "ArmL");
        Extender = hardwareMap.get(Servo.class, "Extender");
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

    }
    public void armUp(){
        ArmL.setPosition(.3);
        ArmR.setPosition(.86);
        armTogged = false;
    }
    public void armDown(){
        ArmL.setPosition(.7);
        ArmR.setPosition(.53);
        armTogged = true;
    }
    public void armToggle(){
        if(armTogged){
            armUp();
        } else{
            armDown();
        }
    }
    public void linearUpHigh(){
        LRP = 0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        if (getRuntime()>= 0.8) {
            linearLeft.setPower(0);
            linearRight.setPower(0);
        }
    }
    public void linearUpSubmersible(){
        LRP = 0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        if (getRuntime() >= 0.5) {
            linearLeft.setPower(0);
            linearRight.setPower(0);
        }
    }
    public void linearDownFull(){
        LRP = -0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        if (getRuntime() >= 0.5) {
            linearLeft.setPower(0);
            linearRight.setPower(0);
        }
    }

    public void clawOpen(){
        Claw.setPosition(0.75);
        clawTogged = true;
    }
    public void clawClosed(){
        Claw.setPosition(0.39);
        clawTogged = false;
    }

    public void extenderForward(){
        Extender.setPosition(0.7);
    }
    public void extenderBack(){
        Extender.setPosition(0.3);
    }
    public void extenderNeutral(){
        Extender.setPosition(0.5);
    }


}
