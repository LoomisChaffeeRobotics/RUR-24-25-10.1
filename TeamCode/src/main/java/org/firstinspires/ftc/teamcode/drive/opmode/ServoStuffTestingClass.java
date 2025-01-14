package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.servoWork;

@TeleOp
public class ServoStuffTestingClass extends OpMode {
    org.firstinspires.ftc.teamcode.servoWork servos;
    double armLposition = 0;

    @Override
    public void init() {
        servos = new servoWork();
        servos.init(hardwareMap);
        servos.armUp();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();
        servos.armUpdate();

    }
// (.368-servo.getposition())/.368
    @Override
    public void loop() {

        if(gamepad1.a){
            servos.armRightManual(0.001);
        } else if (gamepad1.b){
            servos.armRightManual(-0.001);
        }

//        if(gamepad1.x){
//            servos.armDoubleManual(0.001);
//        }
//        if(gamepad1.y) {
//            servos.armDoubleManual(-0.001);
//        }
        ;


        telemetry.addData("armL", servos.ArmL.getPosition());
        telemetry.addData("armR", servos.ArmR.getPosition());
        telemetry.update();
    }
}
