package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE;

import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;

@TeleOp
public class HopefullyFinalDriveClass extends OpMode {
    servoWork servoWork;
    IMU imu;
    SampleMecanumDrive drive;
    Servo Claw;
    Servo ArmR;
    Servo ArmL;
    Servo Extender;
    float speedfactor = 0.002F; //speed at which everything moves
    DcMotor linearRight;
    DcMotor linearLeft;
    double x = 0;
    double y = 0;
    double rx = 0;
    double LRP = 0;
    int cnt = 1;
    int acnt = 1;
    int armTogged = 1;

    int rtdepressed = 0;
    int rbdepressed = 0;



    @Override
    public void init() {
        //init//
        servoWork = new servoWork();
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        drive = new SampleMecanumDrive(hardwareMap);
        ArmR = hardwareMap.get(Servo.class, "ArmR");
        ArmL = hardwareMap.get(Servo.class, "ArmL");
        servoWork.init(hardwareMap);
//         servoWork.armUp();
        Claw = hardwareMap.get(Servo.class, "Claw");
//        servoWork.clawClosed();
        Extender = hardwareMap.get(Servo.class, "Extender");
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        //controls - right trigger = claw, right bumper = arm, left trigger slows drive, dpad controls extender

        if (gamepad1.back) {
            servoWork.clawOpen();
//            servoWork.armUp();



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
        if ((gamepad1.right_trigger >= 0.2 && !gamepad1.left_bumper) && cnt == 2 && rtdepressed == 0) {
            servoWork.clawOpen();
            telemetry.addData("Claw Pos:", (Claw.getPosition()));
            cnt = 1;
            rtdepressed = 1;
        } else if ((gamepad1.right_trigger >= 0.2 && !gamepad1.left_bumper) && cnt == 1 && rtdepressed == 0) {
            servoWork.clawClosed();
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

//        if (gamepad1.right_bumper && gamepad1.x) {
//            ArmR.setPosition(ArmR.getPosition() - speedfactor);
//            ArmL.setPosition(ArmL.getPosition() - speedfactor);
//            telemetry.addData("Arm Pos:", (ArmL.getPosition()));
//        }
//        if (gamepad1.right_bumper && gamepad1.left_bumper) {
//            ArmR.setPosition(ArmR.getPosition() + speedfactor);
//            ArmL.setPosition(ArmL.getPosition() + speedfactor);
//            telemetry.addData("Arm Pos:", (ArmL.getPosition()));
//        }

        if ((gamepad1.right_bumper && !gamepad1.left_bumper && !gamepad1.x && rbdepressed != 1)) {
            if (armTogged == 1) {
//                servoWork.armUp();
                armTogged = 0;
            } else {
                ArmR.setPosition(0.70);
                ArmL.setPosition(0.59);
                armTogged = 1;
            }
            telemetry.addData("ArmR pos,", ArmR.getPosition());
            telemetry.addData("ArmL pos,", ArmL.getPosition());
            rbdepressed = 1;
        }
        if (!gamepad1.right_bumper) {
            rbdepressed = 0;
        }

        if (gamepad1.y && gamepad1.left_bumper) {
            LRP = 0.3;  // LRP is linear slide power
        } else if (gamepad1.y && !gamepad1.left_bumper) {
            linearLeft.setPower(LRP);
            linearRight.setPower(LRP);

//            LRP = 1;
//            servoWork.linearUpHigh();
        }

        if (gamepad1.a && gamepad1.left_bumper) {
            LRP = -0.3;
        } else if (gamepad1.a && !gamepad1.left_bumper) {
//             LRP = -0.7;
//            servoWork.linearDownFull();
        }
        if (gamepad1.dpad_up) {
            servoWork.extenderBack();
        }
        if (gamepad1.dpad_down) {
            servoWork.extenderForward();
        }
        if (!gamepad1.dpad_up && !gamepad1.dpad_down){
            servoWork.extenderNeutral();
        }


        linearRight.setPower(LRP);
        linearLeft.setPower(LRP);






        //Field-centric//
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = (x * Math.cos(-botHeading)) - (y * Math.sin(-botHeading));
        double rotY = (x * Math.sin(-botHeading)) + (y * Math.cos(-botHeading));
        rotX = rotX * 1.10134867899;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        drive.setMotorPowers((rotY + rotX + rx) / denominator,(rotY - rotX + rx) / denominator,(rotY + rotX - rx) / denominator,(rotY - rotX - rx) / denominator);
        // options = start button
        if (gamepad1.back) {
            imu.resetYaw();
        }
        if (gamepad2.back) {
            imu.resetYaw();
        }
        telemetry.addData("Linear Right Pos:", linearRight.getCurrentPosition());
        telemetry.addData("Linear Left Pos:", linearLeft.getCurrentPosition());
        telemetry.addData("heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("Y joystick value", y);
        telemetry.addData("X joystick value", x);
        telemetry.addData("RX joystick value", rx);
        telemetry.addData("rotated X value", rotX);
        telemetry.addData("rotated Y value", rotY);
        telemetry.addData("leftFront power", (rotY+rotX+rx)/denominator);
        telemetry.addData("leftRear power", (rotY-rotX+rx)/denominator);
        telemetry.addData("rightRear power", (rotY+rotX-rx)/denominator);
        telemetry.addData("rightFront power", (rotY-rotX-rx)/denominator);
        telemetry.update();


    }

}

