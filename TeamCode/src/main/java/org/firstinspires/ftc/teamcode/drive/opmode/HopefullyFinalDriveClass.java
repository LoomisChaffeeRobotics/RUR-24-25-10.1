package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp

public class HopefullyFinalDriveClass extends OpMode {
    IMU imu;
    SampleMecanumDrive drive;
    Servo Claw;
    Servo Arm;
    float speedfactor = 0.002F; //speed at which everything moves
    float Clawclose = 0.39F;
    float Clawopen = Clawclose + 0.29F;
    float Armopen = 0.5F;
    float Armclose = Armopen - 0.2F;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;
    DcMotor linearRight;
    DcMotor linearLeft;
    double x = 0;
    double y = 0;
    double rx = 0;
    double n = 0;
    double ns = 0;
    int cnt = 1;
    int acnt = 1;

    int rtdepressed = 0;
    int rbdepressed = 0;



    @Override
    public void init() {
        //init//
        imu = hardwareMap.get(IMU.class, "imu");
        drive = new SampleMecanumDrive(hardwareMap);
        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm.setPosition(Armopen);
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(Clawopen);
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
    }

    @Override
    public void loop() {
        //controls - right trigger = claw, right bumper = arm, left trigger slows drive
        if (gamepad1.back) {
            Claw.setPosition(Clawopen);
            Arm.setPosition(Armopen);

        }
        if ((gamepad1.right_trigger >= 0.2 && !gamepad1.left_bumper) && cnt == 2 && rtdepressed == 0) {
            Claw.setPosition(Clawopen);
            telemetry.addData("Claw Pos:", (Claw.getPosition()));
            cnt = 1;
            rtdepressed = 1;
        }
        else if ((gamepad1.right_trigger >= 0.2 && !gamepad1.left_bumper) && cnt == 1 && rtdepressed == 0) {
            Claw.setPosition(Clawclose);
            telemetry.addData("Claw Pos:", (Claw.getPosition()));
            cnt = 2;
            rtdepressed = 1;
        }
        if (gamepad1.right_trigger < 0.2) {rtdepressed = 0;}

        if (gamepad1.right_bumper && gamepad1.a) {
            Arm.setPosition(Arm.getPosition() - speedfactor);
            telemetry.addData("Arm Pos:", (Arm.getPosition()));
        }
        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            Arm.setPosition(Arm.getPosition() + speedfactor);
            telemetry.addData("Arm Pos:", (Arm.getPosition()));
        }

        if ((gamepad1.right_bumper && !gamepad1.left_bumper) && acnt == 2 && rbdepressed == 0) {
            Arm.setPosition(Armopen);
            telemetry.addData("Arm Pos:", (Arm.getPosition()));
            acnt = 1;
            rbdepressed = 1;
        }
        else if ((gamepad1.right_bumper && !gamepad1.left_bumper) && acnt == 1 && rbdepressed == 0) {
            Arm.setPosition(Armclose);
            telemetry.addData("Arm Pos:", (Arm.getPosition()));
            acnt = 2;
            rbdepressed = 1;
        }
        if (gamepad1.right_bumper) {rbdepressed = 0;}




        if (gamepad1.left_trigger >= 0.3) {
            y = (0.3) * gamepad1.left_stick_y;
        }
        else if (gamepad1.left_trigger <= 0.3){
            y = gamepad1.left_stick_y;
        }

        if (gamepad1.left_trigger >= 0.3) {
            x = (0.3) * gamepad1.left_stick_x;
        }
        else if (gamepad1.left_trigger <= 0.3){
            x = gamepad1.left_stick_x;
        }
// Second person turns while first person drives//
        if (gamepad1.left_trigger >= 0.3) {
            rx = (0.3) * gamepad2.right_stick_x;
        }
        else if (gamepad1.left_trigger <= 0.3){
            rx = gamepad2.right_stick_x;
        }
        //Field-centric//
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1824943423;
        double rx = gamepad2.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        drive.setMotorPowers((rotY + rotX + rx) / denominator,(rotY - rotX + rx) / denominator,(rotY + rotX - rx) / denominator,(rotY - rotX - rx) / denominator);
        // options = start button
        if (gamepad1.options) {
            imu.resetYaw();
        }
        if (gamepad2.options) {
            imu.resetYaw();
        }


    }

}

