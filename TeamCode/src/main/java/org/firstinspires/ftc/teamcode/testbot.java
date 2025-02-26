package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testbot extends OpMode {

    Servo Claw;
    Servo ArmR;
    Servo ArmL;
    Servo Extender;
    float speedfactor = 0.002F;
    float Clawclose = 0.39F;
    float Clawopen = 0.75F;
    float Armopen = 0.8973F;
    float Armclose = 0.5252F;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;
    DcMotor linearRight;
    DcMotor linearLeft;
    double x = 0;
    double y = 0;
    double rx = 0;
    double LRP = 0;
    int cnt = 1;
    int acnt = 1;

    int rtdepressed = 0;
    int rbdepressed = 0;


    @Override
    public void init() {


        ArmR = hardwareMap.get(Servo.class, "ArmR");
        ArmR.setDirection(Servo.Direction.REVERSE);
        //ArmR.setPosition(Armopen);
        ArmL = hardwareMap.get(Servo.class, "ArmL");
        //ArmL.setPosition(Armopen);
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(Clawclose);
        Extender = hardwareMap.get(Servo.class, "Extender");

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotor.class, "rearLeft");
        rearRight = hardwareMap.get(DcMotor.class, "rearRight");
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void loop() {
        if (gamepad1.back) {
            Claw.setPosition(Clawopen);
            //ArmR.setPosition(1-Armopen);
            //ArmL.setPosition((Armopen));

        }


        if ((gamepad1.right_trigger >= 0.2 && !gamepad1.left_bumper) && cnt == 2 && rtdepressed == 0) {
            Claw.setPosition(Clawopen);
            telemetry.addData("Claw Pos:", (Claw.getPosition()));
            cnt = 1;
            rtdepressed = 1;
        } else if ((gamepad1.right_trigger >= 0.2 && !gamepad1.left_bumper) && cnt == 1 && rtdepressed == 0) {
            Claw.setPosition(Clawclose);
            telemetry.addData("Claw Pos:", (Claw.getPosition()));
            cnt = 2;
            rtdepressed = 1;
        }
        if (gamepad1.right_trigger < 0.2) {
            rtdepressed = 0;
        }

        if (gamepad1.right_trigger > 0.2 && gamepad1.x) {
            Claw.setPosition(Claw.getPosition() - speedfactor);
            telemetry.addData("Claw Pos:", (Claw.getPosition()));
        }
        if (gamepad1.right_trigger > 0.2 && gamepad1.left_bumper) {
            Claw.setPosition(Claw.getPosition() + speedfactor);
            telemetry.addData("Claw Pos:", (Claw.getPosition()));
        }

        //if (gamepad1.right_bumper && gamepad1.x) {
        // ArmR.setPosition(ArmR.getPosition() + speedfactor);
        ///ArmL.setPosition(ArmL.getPosition() - speedfactor);
        // telemetry.addData("Arm Pos:", (ArmL.getPosition()));
        //}
        //if (gamepad1.right_bumper && gamepad1.left_bumper) {
        //   ArmR.setPosition(ArmR.getPosition() - speedfactor);
        // ArmL.setPosition(ArmL.getPosition() + speedfactor);
        // telemetry.addData("Arm Pos:", (ArmL.getPosition()));
        //}

        //if ((gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.x) && acnt == 2 && rbdepressed == 0) {
        // ArmR.setPosition(Armopen);
        //  ArmL.setPosition(Armopen);
        // telemetry.addData("Arm Pos:", (ArmL.getPosition()));
        //acnt = 1;
        //  rbdepressed = 1;
        //}
        //else if ((gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.x) && acnt == 1 && rbdepressed == 0) {
        //ArmR.setPosition(Armclose);
        // ArmL.setPosition(Armclose);
        // telemetry.addData("Arm Pos:", (ArmL.getPosition()));
        //acnt = 2;
        //rbdepressed = 1;
        //}
        if (!gamepad1.right_bumper) {
            rbdepressed = 0;
        }


        if (gamepad1.left_trigger >= 0.3) {
            y = (0.3) * -gamepad1.left_stick_y;
        } else if (gamepad1.left_trigger <= 0.3) {
            y = -gamepad1.left_stick_y;
        }

        if (gamepad1.left_trigger >= 0.3) {
            x = (0.3) * gamepad1.left_stick_x;
        } else if (gamepad1.left_trigger <= 0.3) {
            x = gamepad1.left_stick_x;
        }

        if (gamepad1.left_trigger >= 0.3) {
            rx = (0.3) * gamepad1.right_stick_x;
        } else if (gamepad1.left_trigger <= 0.3) {
            rx = gamepad1.right_stick_x;
        }

        if (gamepad1.y && gamepad1.left_bumper) {
            LRP = 0.3;
        } else if (gamepad1.y && !gamepad1.left_bumper) {
            LRP = 1;
        }

        if (gamepad1.a && gamepad1.left_bumper) {
            LRP = -0.3;
        } else if (gamepad1.a && !gamepad1.left_bumper) {
            LRP = -0.7;
        }

        if (gamepad1.dpad_up) {
            Extender.setPosition(0.3);
        }
        if (gamepad1.dpad_down) {
            Extender.setPosition(0.7);
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
            Extender.setPosition(0.5);
        }


        frontLeft.setPower(y + x + rx);
        rearLeft.setPower(y - x + rx);
        frontRight.setPower(y - x - rx);
        rearRight.setPower(y + x - rx);
        linearRight.setPower(LRP);
        linearLeft.setPower(LRP);
        y = 0;
        x = 0;
        rx = 0;
        LRP = 0.1;

    }
}