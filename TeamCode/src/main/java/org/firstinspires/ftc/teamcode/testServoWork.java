package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class testServoWork extends OpMode {
    servoWork servos;
    @Override
    public void init() {
        servos = new servoWork();
        servos.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            servos.linearUpHigh();
        }
        if(gamepad1.b){
            servos.linearDownFullFromHigh();
        }
        if(gamepad1.x){
            servos.clawOpen();
        }
        if(gamepad1.y){
            servos.clawClosed();
        }
        if(gamepad1.dpad_down){
            servos.extenderBack();
        }
        if(gamepad1.dpad_up){
            servos.extenderForward();
        }
        if(gamepad1.dpad_right){
            servos.extenderNeutral();
        }
        telemetry.addData("time ", servos.elapsedTime.time());
    }
}
