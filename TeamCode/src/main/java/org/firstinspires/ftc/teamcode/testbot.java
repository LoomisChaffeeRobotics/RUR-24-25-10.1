package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testbot extends OpMode {

    Servo Claw;
    Servo Arm;
    CRServo Extender;
    float speedfactor = 0.002F;
    float Clawclose = 0.39F;
    float Clawopen = 0.75F;
    float Armopen = 0.8973F;
    float Armclose = Armopen - 0.3531F;
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


        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm.setPosition(Armopen);
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(Clawclose);
        Extender = hardwareMap.get(CRServo.class, "Extender");

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
            Arm.setPosition(Armopen);

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

            if (gamepad1.right_bumper && gamepad1.x) {
                Arm.setPosition(Arm.getPosition() - speedfactor);
                telemetry.addData("Arm Pos:", (Arm.getPosition()));
            }
            if (gamepad1.right_bumper && gamepad1.left_bumper) {
                Arm.setPosition(Arm.getPosition() + speedfactor);
                telemetry.addData("Arm Pos:", (Arm.getPosition()));
            }

            if ((gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.x) && acnt == 2 && rbdepressed == 0) {
                Arm.setPosition(Armopen);
                telemetry.addData("Arm Pos:", (Arm.getPosition()));
                acnt = 1;
                rbdepressed = 1;
            }
            else if ((gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.x) && acnt == 1 && rbdepressed == 0) {
                Arm.setPosition(Armclose);
                telemetry.addData("Arm Pos:", (Arm.getPosition()));
                acnt = 2;
                rbdepressed = 1;
            }
            if (!gamepad1.right_bumper) {rbdepressed = 0;}


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

        if (gamepad1.b) {
            Extender.setPower(-0.4);
        }
        if (gamepad1.x) {
            Extender.setPower(0.4);
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
            Extender.setPower(0);
        }

}