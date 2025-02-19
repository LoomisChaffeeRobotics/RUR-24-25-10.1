package org.firstinspires.ftc.teamcode.drive.AutonModes;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous
@Config

public class AutonomousFinalSample extends LinearOpMode {
    IMU imu;
    ElapsedTime timer;
    ElapsedTime timer2;
    SampleMecanumDrive drive;
    servoWork servos;
    boolean startedDriving = false;
    int parkSide = -1;
    TrajectorySequence specimenSide;
    TrajectorySequence specimenSide2; //post localization

    final double upTime = 2.17d;
    final double downTime = 1.7d;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        servos = new servoWork();
        servos.init(hardwareMap);
        Pose2d startPose = new Pose2d(-24, -64.5, Math.toRadians(90));
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-24,-64.5, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(-50,-47), Math.toRadians(90))
                .addTemporalMarker(1.34, () -> {
                    servos.clawOpen();
                    servos.armDownAuto();
                })
                .waitSeconds(1.79)
                .addTemporalMarker(3.13, () -> {
                    servos.clawClosed(); // need to be open/closed not toggle
                })
                .waitSeconds(0.67)
                .addTemporalMarker(3.80, () -> {
                    servos.armUpAuto();
                })
                .splineToConstantHeading(new Vector2d(-56,-58), Math.toRadians(45))
                .turn(Math.toRadians(135))
                .addTemporalMarker(6.80, () -> {
                    servos.linearPower(1);
                })
                .waitSeconds(upTime)
                .addTemporalMarker(8.97, () -> {
                    servos.linearPower(0.12);
                })
                .addTemporalMarker(9.3, () -> {
                    servos.arm45Auto();
                })
                .waitSeconds(1.5)
                .addTemporalMarker(10.8, () -> {
                    servos.clawOpen();
                    servos.armUpAuto();
                    servos.linearPower(-1);
                })
                .waitSeconds(downTime)
                .addTemporalMarker(13.26, () -> {
                    servos.linearPower(0);
                })
                .splineToConstantHeading(new Vector2d(-61,-48.5), Math.toRadians(90))
                .turn(Math.toRadians(-135))
                .addTemporalMarker(14.98, () -> {
                    servos.armDownAuto();
                })
                .waitSeconds(1.5)
                .addTemporalMarker(16.48, () -> {
                    servos.clawClosed();
                })
                .waitSeconds(1)
                .addTemporalMarker(17.48, () -> {
                    servos.armUpAuto();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-54,-56), Math.toRadians(45))
                .turn(Math.toRadians(135))
                .addTemporalMarker(22.37, () -> {
                    servos.linearPower(1);
                })
                .waitSeconds(upTime)
                .addTemporalMarker(24.54, () -> {
                    servos.linearPower(0.12);
                })
                .addTemporalMarker(24.8, () -> {
                    servos.arm45Auto();
                })
                .waitSeconds(1.5)
                .addTemporalMarker(26.3, () -> {
                    servos.clawOpen();
                    servos.armUpAuto();
                    servos.linearPower(-1);
                })
                .waitSeconds(downTime)
                .addTemporalMarker(29, () -> {
                    servos.linearPower(0);
                })
                .splineToConstantHeading(new Vector2d(-59,-42.5), Math.toRadians(90))
                .turn(Math.toRadians(-100))
                .addTemporalMarker(30.5, () -> {
                    servos.armDownAuto();
                    servos.clawAuto();
                })
                .forward(3.5)
                .addTemporalMarker(32, () -> {
                    servos.clawClosed();
                })
                .waitSeconds(1)
                .addTemporalMarker(33, () -> {
                    servos.armUpAuto();
                })
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(-51.5,-53.5), Math.toRadians(45))
                .turn(Math.toRadians(100))
                .strafeRight(3)
                .addTemporalMarker(38, () -> {
                    servos.linearPower(1);
                })
                .waitSeconds(upTime)
                .addTemporalMarker(40.17, () -> {
                            servos.linearPower(0.12);
                })
                .addTemporalMarker(40.5, () -> {
                    servos.arm45Auto();
                })
                .waitSeconds(1.5)
                .addTemporalMarker(42, () -> {
                    servos.clawOpen();
                    servos.armUpAuto();
                    servos.linearPower(-1);
                })
                .waitSeconds(downTime)
                .addTemporalMarker(44, () -> {
                    servos.linearPower(0);
                })
//
                .build();
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequenceAsync(trajSeq);


        waitForStart();


        if (isStopRequested()) return;
        while (opModeIsActive()) {

            drive.update(); // this is here
            telemetry.addData("armState", servos.armDowning);
            telemetry.addData("armUpdate", servos.isArmRunning());
            telemetry.addData("poseEstimate", drive.getPoseEstimate());
            telemetry.update();


        }

    }
    void update(){
        servos.armUpdate();
        while (servos.isArmRunning()) {
            servos.armUpdate();
        }
        }

}

