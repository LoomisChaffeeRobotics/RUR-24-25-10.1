package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    @Override
    public void init() {
        servo2 = hardwareMap.get(Servo.class, "ArmR");
        servo1 = hardwareMap.get(Servo.class, "Claw");
        servo3 = hardwareMap.get(Servo.class, "ArmL");
        servo4 = hardwareMap.get(Servo.class, "Extender");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        rightRear = hardwareMap.get(DcMotor.class, "rearRight");
        leftRear = hardwareMap.get(DcMotor.class, "rearLeft");
    }

    @Override
    public void loop() {
        telemetry.addData("ArmL pos: ", servo1.getPosition());
        telemetry.addData("Extender pos: ", servo2.getPosition());
        if (gamepad1.x){
            rightFront.setPower(1);
        } else {
            rightFront.setPower(0);

        }
        if (gamepad1.b) {
            leftFront.setPower(1);
        } else {
            leftFront.setPower(0);
        }
        if (gamepad1.y){
            leftRear.setPower(1);
        } else {
            leftRear.setPower(0);
        }
        if (gamepad1.a){
            rightRear.setPower(1);
        } else {
            rightRear
                    .setPower(0);
        }

        telemetry.addData("RR", rightRear.getCurrentPosition());
        telemetry.addData("LR", leftRear.getCurrentPosition());
        telemetry.addData("RF", rightFront.getCurrentPosition());
        telemetry.addData("LF", leftFront.getCurrentPosition());



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
