package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class servoWork extends OpMode{
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor leftRear;
    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
    boolean armTogged = true;
    IMU imu;
    SampleMecanumDrive drive;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        imu = hardwareMap.get(IMU.class,"imu");
        servo2 = hardwareMap.get(Servo.class, "ArmR");
        servo1 = hardwareMap.get(Servo.class, "Claw");
        servo3 = hardwareMap.get(Servo.class, "ArmL");
        servo4 = hardwareMap.get(Servo.class, "Extender");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightRear = hardwareMap.get(DcMotor.class, "rearRight");
        leftRear = hardwareMap.get(DcMotor.class, "rearLeft");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
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
        leftFront.setPower((rotY + rotX + rx) / denominator);
        leftRear.setPower((rotY - rotX + rx) / denominator);
        rightFront.setPower((rotY - rotX - rx) / denominator);
        rightRear.setPower((rotY + rotX - rx)/ denominator);
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


    }
    public void armUp(){
        servo1.setPosition(.3);
        servo2.setPosition(.86);
        armTogged = false;
    }
    public void armDown(){
        servo1.setPosition(.7);
        servo2.setPosition(.53);
        armTogged = true;
    }
    public void armToggle(){
        if(armTogged){
            armUp();
        } else{
            armDown();
        }


    }



}
