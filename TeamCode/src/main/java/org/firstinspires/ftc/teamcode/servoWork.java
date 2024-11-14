package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoWork extends OpMode{
    Servo servo1;
    Servo servo2;
    boolean armTogged = true;
    boolean aPressed = false;
    @Override
    public void init() {
        servo2 = hardwareMap.get(Servo.class, "ArmR");
        servo1 = hardwareMap.get(Servo.class, "Claw");
    }

    @Override
    public void loop() {
        telemetry.addData("claw pos: ", servo1.getPosition());
        telemetry.addData("ArmR pos: ", servo2.getPosition());
        if (gamepad1.x){
            servo1.setPosition(.7);
            servo2.setPosition(.53);

        }
        if (gamepad1.b) {
            servo1.setPosition(.3);
            servo2.setPosition(.86);
        }
        if (gamepad1.y){
            servo2.setPosition(servo2.getPosition()+0.001);
        }
        if (gamepad1.a){
            if (aPressed) {
                armToggle();
                aPressed = true;
            }

        }else {
            aPressed=false;
        }
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
