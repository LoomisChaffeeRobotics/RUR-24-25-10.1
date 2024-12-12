package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    Servo Extender;
    DcMotor linearRight;
    DcMotor linearLeft;
    boolean armTogged = true;
    boolean armTouched;
    boolean clawTogged = true;
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
        Extender = hardwareMap.get(Servo.class, "Extender");
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
<<<<<<< Updated upstream
=======
//        telemetry.addData("ArmL pos: ", servo3.getPosition());
//        telemetry.addData("Extender pos: ", servo4.getPosition());
//        if (gamepad1.x){
//            rightFront.setPower(1);
//        } else {
//            rightFront.setPower(0);
//
//        }
//        if (gamepad1.b) {
//            leftFront.setPower(1);
//        } else {
//            leftFront.setPower(0);
//        }
//        if (gamepad1.y){
//            leftRear.setPower(1);
//        } else {
//            leftRear.setPower(0);
//        }
//        if (gamepad1.a){
//            rightRear.setPower(1);
//        } else {
//            rightRear
//                    .setPower(0);
//        }


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;//CHANGE ASAP//;
        double rx = gamepad1.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotX = rotX * 1.1824943423;
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFront.setPower((y + x + rx) / denominator);
        leftRear.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightRear.setPower((y + x - rx)/ denominator);
        // options = start button

        if (gamepad1.options) {
            imu.resetYaw();
        }
        if (gamepad2.options) {
            imu.resetYaw();
        }
        telemetry.addData("RR power", rightRear.getPower());
        telemetry.addData("LR power", leftRear.getPower());
        telemetry.addData("RF power", rightFront.getPower());
        telemetry.addData("LF power", leftFront.getPower());


>>>>>>> Stashed changes
    }

    public void armUp(){
//        ArmL.setPosition(.3);
        ArmR.setPosition(.70);
        armTogged = false;
    }
    public void armDown(){
//        ArmL.setPosition(.7);
        ArmR.setPosition(.38);
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
        elapsedTime.reset(); // we are not using time. BAD IDEA TODO: change this
        linearRight.setTargetPosition(30);
        linearLeft.setTargetPosition(-30);
    }
    public void linearUpSubmersible(){
        LRP = 0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        elapsedTime.reset();
        if (elapsedTime.time() >= 0.5) {
            linearLeft.setPower(0);
            linearRight.setPower(0);
        }
        }
    public void linearDownFull(){
        LRP = -0.7;
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);
        elapsedTime.reset();
        if (elapsedTime.time() >= 0.5) {
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
