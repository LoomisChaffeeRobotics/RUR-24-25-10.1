package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class AutonomousTest extends OpMode {

    IMU imu;
    SampleMecanumDrive drive;
    Servo Claw;
    Servo Arm;
    float speedfactor = 0.002F; //speed at which everything moves
    float Clawclose = 0.39F;
    float Clawopen = 0.75F;
    float Armopen = 0.8573F;
    float Armclose = Armopen - 0.5042F;
    DcMotor linearRight;
    DcMotor linearLeft;


    @Override
    public void init() {

        imu = hardwareMap.get(IMU.class, "imu");
        drive = new SampleMecanumDrive(hardwareMap);
        Arm = hardwareMap.get(Servo.class, "Arm");
        Arm.setPosition(Armopen);
        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setPosition(Clawclose);
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        linearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    @Override
    public void loop() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory Auto1 = drive.trajectoryBuilder(new Pose2d())
                .forward(30)
                .build();
        drive.followTrajectory(Auto1);
        Trajectory Auto2 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(30, () -> {
                    // this is to copy paste some code for auto test
                })
                .strafeRight(36)
                .forward(24)
                .strafeRight(12)
                .back(30)
                .forward(30)
                .strafeRight(9)
                .back(30)
                .forward(30)
                .strafeRight(3)
                .back(30)
                .build();
        Trajectory Auto3 = drive.trajectoryBuilder(new Pose2d())
                .addDisplacementMarker(30, () -> {
                    // this is to copy paste the samem code for auto test

                })
                .strafeLeft(36)
                .forward(24)
                .strafeLeft(12)
                .back(60)
                .forward(60)
                .strafeLeft(12)
                .back(60)
                .forward(60)
                .strafeLeft(12)
                .back(60)
                .build();



        drive.followTrajectory(Auto1);

        drive.followTrajectory(Auto2);

        drive.turn(Math.toRadians(1234));


    }

}
