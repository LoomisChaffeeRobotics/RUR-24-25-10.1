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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
@Config

public class AutonomousFinalSample extends LinearOpMode {
    IMU imu;
    TrajectorySequenceBuilder TrajectorySequenceBuilder;
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
                .lineTo(new Vector2d(-50,-46))
                .addTemporalMarker(0.7, () -> {
                    servos.clawOpen();
                    servos.armDownAuto();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    servos.clawClosed(); // need to be open/closed not toggle
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    servos.armUpAuto();
                })
                .waitSeconds(0.7)
                .lineToLinearHeading(new Pose2d(-51.5,-53.5, Math.toRadians(235)))
                .addTemporalMarker(() -> {
                    servos.linearPower(1);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    servos.linearPower(0.12);
                })
                .addTemporalMarker(() -> {
                    servos.arm45Auto();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    servos.clawOpen();
                    servos.armUpAuto();
                    servos.linearPower(-1);
                })
                .waitSeconds(downTime)
                .addTemporalMarker(() -> {
                    servos.linearPower(0);
                })
                .lineToSplineHeading(new Pose2d(-59,-45, Math.toRadians(90)))
                .addTemporalMarker(() -> {
                    servos.armDownAuto();
                })
                .waitSeconds(0.6)
                .addTemporalMarker( () -> {
                    servos.clawClosed();
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    servos.armUpAuto();
                })
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(-52.5,-52.5, Math.toRadians(230)))
                .addTemporalMarker(() -> {
                    servos.linearPower(1);
                })
                .waitSeconds(upTime)
                .addTemporalMarker(() -> {
                    servos.linearPower(0.12);
                })
                .addTemporalMarker(() -> {
                    servos.arm45Auto();
                })
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    servos.clawOpen();
                    servos.armUpAuto();
                    servos.linearPower(-1);
                })
                .waitSeconds(downTime)
                .addTemporalMarker(() -> {
                    servos.linearPower(0);
                })
//                .lineToLinearHeading(new Pose2d(-58,-43.5, Math.toRadians(125)))
//                .addTemporalMarker(() -> {
//                    servos.armDownAuto();
//                    servos.clawAuto();
//                })
//                .waitSeconds(0.5)
//                .forward(1)
//                .addTemporalMarker(() -> {
//                    servos.clawClosed();
//                })
//                .waitSeconds(0.3)
//                .addTemporalMarker(() -> {
//                    servos.armUpAuto();
//                })
//                .waitSeconds(0.6)
//                .lineToLinearHeading(new Pose2d(-53.5,-53.5, Math.toRadians(230)))
//                .addTemporalMarker(() -> {
//                    servos.linearPower(1);
//                })
//                .waitSeconds(upTime)
//                .addTemporalMarker(() -> {
//                    servos.linearPower(0.12);
//                    servos.arm45Auto();
//                })
//                .waitSeconds(0.6)
//                .addTemporalMarker(() -> {
//                    servos.clawOpen();
//                    servos.armUpAuto();
//                    servos.linearPower(-1);
//                })
//                .waitSeconds(downTime)
//                .addTemporalMarker(() -> {
//                    servos.linearPower(0);
//                })
                .turn(Math.toRadians(-135))
ync(trajSeq);


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

