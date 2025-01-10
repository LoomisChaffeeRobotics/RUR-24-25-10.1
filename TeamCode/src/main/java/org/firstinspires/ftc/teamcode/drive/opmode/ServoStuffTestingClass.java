package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.servoWork;

@TeleOp
public class ServoStuffTestingClass extends OpMode {
    org.firstinspires.ftc.teamcode.servoWork servos;

    @Override
    public void init() {
        servos = new servoWork();
        servos.init(hardwareMap);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            servos.clawManual(0.001);
        } else if (gamepad1.b){
            servos.clawManual(-0.001);
        }

        if(gamepad1.x){
            servos.armRightManual(0.001);
        }
        if(gamepad1.y) {
            servos.armRightManual(-0.001);
        }


        telemetry.addData("clawServo", servos.Claw.getPosition());
        telemetry.addData("armR", servos.ArmR.getPosition());
        telemetry.update();
    }
}
